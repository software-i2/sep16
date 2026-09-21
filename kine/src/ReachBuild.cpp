// Copyright by BeeX [2026]

#include <kine/ForwardKinematics.h>
#include <kine/InverseKinematics.h>
#include <kine/ReachBuild.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace kine {
namespace {

void requireOneBranch(const ArmModel &model, int joint) {
    if (model.upperLimit(joint) - model.lowerLimit(joint) >= 2.0 * M_PI) {
        throw std::invalid_argument(std::string("the ") + JOINT_KEYS[joint]
                                    + " window spans a whole turn, so a reach table cannot be exact");
    }
}

// rigidLowest ignores the blades, and the blades are what a posture costs to place, so the
// per-cell call asks for as few of them as pose() will give. The jaw is accounted for
// separately by the drop table, which does not depend on where the cell is.
constexpr size_t kRigidOnly = size_t(1) << 30;

double rigidLowest(const BodyPose &pose) {
    double lowest = std::min(pose.throat.z(), pose.tip.z());
    for (const Segment &link : pose.links) {
        lowest = std::min(lowest, std::min(link.start.z(), link.end.z()));
    }
    return lowest;
}

// The blade points sit at jaw_mount plus a combination of the jaw axes, and none of those
// axes carries the base angle into z. So how far the blades hang below the jaw mount depends
// only on the wrist axis and the roll, never on where the cell is.
std::vector<double> buildBladeDrops(const ArmBody &body, const ReachSpec &spec, double axis_lo, double axis_hi) {
    const ArmModel &model      = body.model();
    const double    roll_lo    = model.lowerLimit(WRIST);
    const double    roll_hi    = model.upperLimit(WRIST);
    const double    axis_span  = axis_hi - axis_lo;
    const double    roll_span  = roll_hi - roll_lo;
    BodyPose        pose;

    std::vector<double> drops(static_cast<size_t>(spec.axis_samples), 0.0);
    for (int i = 0; i < spec.axis_samples; ++i) {
        const double axis = axis_lo + axis_span * i / (spec.axis_samples - 1);
        double       best = -std::numeric_limits<double>::infinity();
        for (int r = 0; r < spec.roll_samples; ++r) {
            const double roll = spec.roll_samples == 1 ? roll_lo : roll_lo + roll_span * r / (spec.roll_samples - 1);
            JointAngles  joints = {{0.0, axis, 0.0, roll}};
            body.pose(joints, pose);
            const double mount = pose.links[WRIST_TUBE].end.z();
            double       drop  = 0.0;
            for (const Eigen::Vector3d &point : pose.blade_points) {
                drop = std::min(drop, point.z() - mount);
            }
            best = std::max(best, drop);
        }
        drops[static_cast<size_t>(i)] = best;
    }
    return drops;
}

double dropAt(const std::vector<double> &drops, double axis_lo, double axis_hi, double axis) {
    if (drops.empty()) {
        return 0.0;
    }
    const size_t last = drops.size() - 1;
    const double step = (axis_hi - axis_lo) / last;
    const double slot = (axis - axis_lo) / step;
    const long   low  = std::max(0L, std::min(static_cast<long>(last), static_cast<long>(std::floor(slot))));
    const long   high = std::max(0L, std::min(static_cast<long>(last), low + 1));
    return std::max(drops[static_cast<size_t>(low)], drops[static_cast<size_t>(high)]);
}

}  // namespace

void buildReachTable(const ArmBody &body, const ArmConfig &config, const ReachSpec &spec, ReachTable &out) {
    const ArmModel &model = body.model();
    requireOneBranch(model, BASE);
    requireOneBranch(model, SHOULDER);
    requireOneBranch(model, ELBOW);
    if (spec.cell <= 0.0) {
        throw std::invalid_argument("the reach table cell size must be above zero");
    }
    if (spec.roll_samples > 0 && spec.axis_samples < 2) {
        throw std::invalid_argument("a reach table that models the jaw blades needs at least two axis samples");
    }

    const ArmDimensions &d       = model.dimensions();
    const PlanarLink     upper   = model.upperArm();
    const PlanarLink     forearm = model.forearmTo(spec.along);
    const double         span    = upper.length + forearm.length;
    const double         fold    = std::fabs(upper.length - forearm.length);

    ReachSpec grid  = spec;
    grid.radial_min = 0.0;
    grid.z_min      = spec.floor_z;
    const double radial_max = span + d.base_axis_to_shoulder_x;
    const double z_max      = d.base_to_base_axis_z + d.base_axis_to_shoulder_z + span;
    grid.cells_r    = static_cast<int>(std::ceil((radial_max - grid.radial_min) / spec.cell)) + 1;
    grid.cells_z    = static_cast<int>(std::ceil((z_max - grid.z_min) / spec.cell)) + 1;
    if (grid.cells_r < 1 || grid.cells_z < 1) {
        throw std::invalid_argument("the arm reaches nothing above the safety floor");
    }

    out.reset(grid, ReachTable::hashConfig(config, spec), model.lowerLimit(BASE), model.upperLimit(BASE));

    const double elbow_sign = model.elbowSign();
    const double axis_lo    = model.lowerLimit(SHOULDER)
                           + std::min(elbow_sign * model.lowerLimit(ELBOW), elbow_sign * model.upperLimit(ELBOW));
    const double axis_hi = model.upperLimit(SHOULDER)
                           + std::max(elbow_sign * model.lowerLimit(ELBOW), elbow_sign * model.upperLimit(ELBOW));
    const std::vector<double> drops =
            spec.roll_samples > 0 ? buildBladeDrops(body, grid, axis_lo, axis_hi) : std::vector<double>();

    const double probe = wrapToPi(0.5 * (model.lowerLimit(BASE) + model.upperLimit(BASE)));
    const double cp    = std::cos(probe);
    const double sp    = std::sin(probe);

    const JointAngles seed = {{0.0, 0.0, 0.0, 0.0}};
    BodyPose          pose;

    for (int iz = 0; iz < grid.cells_z; ++iz) {
        const double z = grid.z_min + (iz + 0.5) * grid.cell;
        for (int ir = 0; ir < grid.cells_r; ++ir) {
            const double    radial = grid.radial_min + (ir + 0.5) * grid.cell;
            ReachCell      &cell   = out.cells()[ReachTable::cellIndex(ir, iz, grid.cells_r)];
            Eigen::Vector3d target(radial * cp, radial * sp, z);

            const double planar_x = radial - d.base_axis_to_shoulder_x;
            const double planar_z = z - d.base_to_base_axis_z - d.base_axis_to_shoulder_z;
            const double distance = std::hypot(planar_x, planar_z);

            for (int b = 0; b < K_REACH_BRANCHES; ++b) {
                JointAngles joints;
                if (solvePosition(model, target, spec.along, ReachTable::branchElbowUp(b), seed, joints)
                    != IkResult::SOLVED) {
                    continue;
                }
                joints[BASE]  = 0.0;
                joints[WRIST] = 0.0;

                const JawAxes axes    = jawAxes(model, joints);
                const double  heading = std::atan2(axes.approach.x(), axes.approach.z());

                const double to_far   = span - distance;
                const double to_fold  = distance - fold;
                const double to_shoulder = std::min(joints[SHOULDER] - model.lowerLimit(SHOULDER),
                                                    model.upperLimit(SHOULDER) - joints[SHOULDER])
                                           * span;
                const double to_elbow = std::min(joints[ELBOW] - model.lowerLimit(ELBOW),
                                                 model.upperLimit(ELBOW) - joints[ELBOW])
                                        * forearm.length;
                const double margin = std::max(0.0, std::min(std::min(to_far, to_fold), std::min(to_shoulder, to_elbow)));

                body.pose(joints, pose, kRigidOnly);
                double lowest = rigidLowest(pose);
                if (!drops.empty()) {
                    const double axis = elbow_sign * joints[ELBOW] + joints[SHOULDER];
                    lowest = std::min(lowest, pose.links[WRIST_TUBE].end.z() + dropAt(drops, axis_lo, axis_hi, axis));
                }

                ReachBranch &branch = cell.branch[b];
                branch.shoulder     = static_cast<float>(joints[SHOULDER]);
                branch.elbow        = static_cast<float>(joints[ELBOW]);
                branch.approach     = static_cast<float>(heading);
                branch.margin       = static_cast<float>(margin);
                branch.lowest_z     = static_cast<float>(lowest);
            }
        }
    }
}

}  // namespace kine
