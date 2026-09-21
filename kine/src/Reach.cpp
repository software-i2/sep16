// Copyright by BeeX [2026]

#include <kine/ForwardKinematics.h>
#include <kine/Reach.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>

namespace kine {
namespace {

const char     kMagic[4]     = {'S', 'R', 'C', 'H'};
const uint32_t kFormatVer    = 1;
const uint64_t kHashBasis    = 1469598103934665603ULL;
const uint64_t kHashPrime    = 1099511628211ULL;

void mixBytes(uint64_t &hash, const void *data, size_t size) {
    const unsigned char *bytes = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < size; ++i) {
        hash ^= bytes[i];
        hash *= kHashPrime;
    }
}

void mix(uint64_t &hash, double value) {
    const double canonical = value == 0.0 ? 0.0 : value;
    mixBytes(hash, &canonical, sizeof(canonical));
}

void mix(uint64_t &hash, int value) { mixBytes(hash, &value, sizeof(value)); }

void mixAngles(uint64_t &hash, const JointAngles &angles) {
    for (int j = 0; j < JOINT_COUNT; ++j) {
        mix(hash, angles[j]);
    }
}

template <typename T>
bool readPod(std::istream &in, T &value) {
    in.read(reinterpret_cast<char *>(&value), sizeof(value));
    return static_cast<bool>(in);
}

template <typename T>
void writePod(std::ostream &out, const T &value) {
    out.write(reinterpret_cast<const char *>(&value), sizeof(value));
}

void writeSpec(std::ostream &out, const ReachSpec &spec) {
    writePod(out, spec.along);
    writePod(out, spec.floor_z);
    writePod(out, spec.cell);
    writePod(out, spec.blade_sample_step);
    writePod(out, spec.roll_samples);
    writePod(out, spec.axis_samples);
    writePod(out, spec.radial_min);
    writePod(out, spec.z_min);
    writePod(out, spec.cells_r);
    writePod(out, spec.cells_z);
}

bool readSpec(std::istream &in, ReachSpec &spec) {
    return readPod(in, spec.along) && readPod(in, spec.floor_z) && readPod(in, spec.cell)
           && readPod(in, spec.blade_sample_step) && readPod(in, spec.roll_samples)
           && readPod(in, spec.axis_samples) && readPod(in, spec.radial_min) && readPod(in, spec.z_min)
           && readPod(in, spec.cells_r) && readPod(in, spec.cells_z);
}

bool sameSpec(const ReachSpec &a, const ReachSpec &b) {
    return a.along == b.along && a.floor_z == b.floor_z && a.cell == b.cell
           && a.blade_sample_step == b.blade_sample_step && a.roll_samples == b.roll_samples
           && a.axis_samples == b.axis_samples && a.radial_min == b.radial_min && a.z_min == b.z_min
           && a.cells_r == b.cells_r && a.cells_z == b.cells_z;
}

}  // namespace

uint64_t ReachTable::hashConfig(const ArmConfig &config, const ReachSpec &spec) {
    uint64_t             hash = kHashBasis;
    const ArmDimensions &d    = config.dimensions;
    mix(hash, d.base_to_base_axis_z);
    mix(hash, d.base_axis_to_shoulder_x);
    mix(hash, d.base_axis_to_shoulder_z);
    mix(hash, d.shoulder_to_elbow_x);
    mix(hash, d.shoulder_to_elbow_z);
    mix(hash, d.elbow_frame_yaw);
    mix(hash, d.elbow_to_wrist_x);
    mix(hash, d.elbow_to_wrist_z);
    mix(hash, d.wrist_to_jaw_mount);
    mix(hash, d.jaw_mount_to_throat);
    mix(hash, d.jaw_mount_to_tip);
    mix(hash, d.hinge_roll_at_wrist_zero);

    mixAngles(hash, config.convention.zero_offset);
    mixAngles(hash, config.convention.direction_sign);
    mixAngles(hash, config.limits.min);
    mixAngles(hash, config.limits.max);

    mix(hash, config.jaw.palm_length);
    mix(hash, config.jaw.open_width);
    mix(hash, config.jaw.blade_rotation_per_metre);
    mix(hash, config.jaw.hinge_offset_closing);
    mix(hash, config.jaw.hinge_offset_approach);
    mix(hash, static_cast<int>(config.jaw.blade_profile.size()));
    for (const BladeBand &band : config.jaw.blade_profile) {
        mix(hash, band.approach_min);
        mix(hash, band.approach_max);
        mix(hash, band.closing_min);
        mix(hash, band.closing_max);
        mix(hash, band.hinge_half_width);
    }

    mix(hash, spec.along);
    mix(hash, spec.floor_z);
    mix(hash, spec.cell);
    mix(hash, spec.blade_sample_step);
    mix(hash, spec.roll_samples);
    mix(hash, spec.axis_samples);
    return hash;
}

void ReachTable::reset(const ReachSpec &spec, uint64_t hash, double base_lower, double base_upper) {
    spec_       = spec;
    hash_       = hash;
    base_lower_ = base_lower;
    base_upper_ = base_upper;

    ReachBranch unreached;
    unreached.shoulder = std::numeric_limits<float>::quiet_NaN();
    unreached.lowest_z = -std::numeric_limits<float>::infinity();

    ReachCell empty;
    for (int b = 0; b < K_REACH_BRANCHES; ++b) {
        empty.branch[b] = unreached;
    }
    cells_.assign(static_cast<size_t>(spec_.cells_r) * spec_.cells_z, empty);
}

const ReachCell *ReachTable::cellAt(double radial, double z) const {
    if (cells_.empty()) {
        return nullptr;
    }
    const int ir = static_cast<int>(std::floor((radial - spec_.radial_min) / spec_.cell));
    const int iz = static_cast<int>(std::floor((z - spec_.z_min) / spec_.cell));
    if (ir < 0 || iz < 0 || ir >= spec_.cells_r || iz >= spec_.cells_z) {
        return nullptr;
    }
    return &cells_[cellIndex(ir, iz, spec_.cells_r)];
}

bool ReachTable::nearReach(double radial, double z) const {
    static const int kOffsetR[5] = {0, -1, 1, 0, 0};
    static const int kOffsetZ[5] = {0, 0, 0, -1, 1};
    if (cells_.empty()) {
        return false;
    }
    const int ir = static_cast<int>(std::floor((radial - spec_.radial_min) / spec_.cell));
    const int iz = static_cast<int>(std::floor((z - spec_.z_min) / spec_.cell));
    for (int n = 0; n < 5; ++n) {
        const int r = ir + kOffsetR[n];
        const int k = iz + kOffsetZ[n];
        if (r < 0 || k < 0 || r >= spec_.cells_r || k >= spec_.cells_z) {
            continue;
        }
        const ReachCell &cell = cells_[cellIndex(r, k, spec_.cells_r)];
        for (int b = 0; b < K_REACH_BRANCHES; ++b) {
            if (cell.branch[b].reached()) {
                return true;
            }
        }
    }
    return false;
}

bool ReachTable::baseAngleFor(double azimuth, double &base_angle) const {
    for (int turns = -1; turns <= 1; ++turns) {
        const double candidate = azimuth + 2.0 * M_PI * turns;
        if (candidate >= base_lower_ && candidate <= base_upper_) {
            base_angle = candidate;
            return true;
        }
    }
    return false;
}

bool ReachTable::save(const std::string &path, std::string &error) const {
    const std::string temp = path + ".tmp";
    {
        std::ofstream out(temp.c_str(), std::ios::binary | std::ios::trunc);
        if (!out) {
            error = "cannot open " + temp;
            return false;
        }
        out.write(kMagic, sizeof(kMagic));
        writePod(out, kFormatVer);
        writePod(out, hash_);
        writePod(out, base_lower_);
        writePod(out, base_upper_);
        writeSpec(out, spec_);
        const uint64_t count = cells_.size();
        writePod(out, count);
        if (count != 0) {
            out.write(reinterpret_cast<const char *>(&cells_[0]), count * sizeof(ReachCell));
        }
        if (!out) {
            error = "write failed on " + temp;
            return false;
        }
    }

    ReachTable reloaded;
    if (!reloaded.load(temp, hash_, error)) {
        return false;
    }
    if (!sameSpec(reloaded.spec_, spec_) || reloaded.cells_.size() != cells_.size()
        || (!cells_.empty()
            && std::memcmp(&reloaded.cells_[0], &cells_[0], cells_.size() * sizeof(ReachCell)) != 0)) {
        error = "the table did not survive a round trip through " + temp;
        return false;
    }
    if (std::rename(temp.c_str(), path.c_str()) != 0) {
        error = "cannot rename " + temp + " to " + path;
        return false;
    }
    return true;
}

bool ReachTable::load(const std::string &path, uint64_t expect_hash, std::string &error) {
    std::ifstream in(path.c_str(), std::ios::binary);
    if (!in) {
        error = "cannot open " + path;
        return false;
    }
    char magic[4] = {0, 0, 0, 0};
    in.read(magic, sizeof(magic));
    if (!in || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
        error = path + " is not a reach table";
        return false;
    }
    uint32_t version = 0;
    if (!readPod(in, version) || version != kFormatVer) {
        error = path + " is an old reach table";
        return false;
    }
    uint64_t hash = 0;
    if (!readPod(in, hash)) {
        error = path + " ends before its hash";
        return false;
    }
    if (hash != expect_hash) {
        error = path + " was built against different geometry";
        return false;
    }
    double lower = 0.0;
    double upper = 0.0;
    if (!readPod(in, lower) || !readPod(in, upper)) {
        error = path + " ends before its base window";
        return false;
    }
    ReachSpec spec;
    if (!readSpec(in, spec)) {
        error = path + " ends before its grid";
        return false;
    }
    uint64_t count = 0;
    if (!readPod(in, count) || count != static_cast<uint64_t>(spec.cells_r) * spec.cells_z) {
        error = path + " does not hold the grid it declares";
        return false;
    }
    std::vector<ReachCell> cells(static_cast<size_t>(count));
    if (count != 0) {
        in.read(reinterpret_cast<char *>(&cells[0]), count * sizeof(ReachCell));
        if (!in) {
            error = path + " ends inside its cells";
            return false;
        }
    }

    spec_       = spec;
    hash_       = hash;
    base_lower_ = lower;
    base_upper_ = upper;
    cells_.swap(cells);
    return true;
}

Eigen::Vector3d approachDirection(double heading, double base_angle) {
    const double x = std::sin(heading);
    const double z = std::cos(heading);
    return Eigen::Vector3d(x * std::cos(base_angle), x * std::sin(base_angle), z);
}

}  // namespace kine
