// Copyright by BeeX [2026]

#ifndef KINE_REACH_H
#define KINE_REACH_H

#include <kine/ArmModel.h>

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace kine {

// The base is a pure spin about z, so nothing stored here depends on azimuth: the 3D grid
// collapses to a 2D one over (radial, z), and the turn onto the arm plane becomes a base
// window test. Only the two elbow branches solvePosition can produce are stored.
enum { K_REACH_BRANCHES = 2 };

// One posture that puts the grasp point on a cell, in kinematic radians at base angle zero.
// NaN in shoulder marks a branch that cannot reach the cell: there is no separate flag, the
// posture is the answer.
struct ReachBranch {
    float shoulder = 0.0f;
    float elbow    = 0.0f;
    float approach = 0.0f;  // jaw approach heading in the arm plane, from +z towards +x
    float margin   = 0.0f;  // metres the target may move before some limit refuses it
    float lowest_z = 0.0f;  // lowest the body reaches, best case over the wrist roll

    bool reached() const { return !std::isnan(shoulder); }
};

struct ReachCell {
    ReachBranch branch[K_REACH_BRANCHES];
};

// What the stored postures were solved against. Everything here changes a cell, so all of it
// goes into the hash; the grid fields are filled by buildReachTable.
struct ReachSpec {
    double along             = 0.0;  // distance down the wrist axis to the grasp point
    double floor_z           = 0.0;
    double cell              = 0.0;
    double blade_sample_step = 0.0;
    int    roll_samples      = 0;  // 0 leaves the jaw blades out of lowest_z
    int    axis_samples      = 0;

    double radial_min = 0.0;
    double z_min      = 0.0;
    int    cells_r    = 0;
    int    cells_z    = 0;
};

class ReachTable {
public:
    static bool   branchElbowUp(int index) { return index != 0; }
    static size_t cellIndex(int ir, int iz, int cells_r) { return static_cast<size_t>(iz) * cells_r + ir; }

    // Covers everything that moves a stored posture. joint_cost_weights are left out on
    // purpose: they steer planner cost, not where the grasp point can go.
    static uint64_t hashConfig(const ArmConfig &config, const ReachSpec &spec);

    const ReachSpec &spec() const { return spec_; }
    uint64_t         hash() const { return hash_; }
    bool             empty() const { return cells_.empty(); }

    // Cell containing (radial, z), or null outside the grid.
    const ReachCell *cellAt(double radial, double z) const;

    // Whether any branch of the cell or its four neighbours reached. Optimistic on purpose:
    // a false negative discards a parking pose with nothing able to recover it, while a
    // false positive costs one exact check that then correctly refuses.
    bool nearReach(double radial, double z) const;

    // The base angle that turns the arm plane onto this azimuth, mirroring what
    // solvePosition does; false when the base window cannot reach it.
    bool baseAngleFor(double azimuth, double &base_angle) const;

    bool save(const std::string &path, std::string &error) const;
    bool load(const std::string &path, uint64_t expect_hash, std::string &error);

    // Sizes the grid and clears every branch to unreached. The generator's entry point.
    void reset(const ReachSpec &spec, uint64_t hash, double base_lower, double base_upper);

    std::vector<ReachCell>       &cells() { return cells_; }
    const std::vector<ReachCell> &cells() const { return cells_; }

private:
    ReachSpec              spec_;
    uint64_t               hash_       = 0;
    double                 base_lower_ = 0.0;
    double                 base_upper_ = 0.0;
    std::vector<ReachCell> cells_;
};

// A stored approach heading turned about the base axis into the arm base frame.
Eigen::Vector3d approachDirection(double heading, double base_angle);

}  // namespace kine

#endif  // KINE_REACH_H
