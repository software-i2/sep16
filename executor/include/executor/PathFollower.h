// Copyright by BeeX [2026]

#ifndef EXECUTOR_PATHFOLLOWER_H
#define EXECUTOR_PATHFOLLOWER_H

#include <executor/ExecutorConfig.h>

#include <vector>

namespace executor {

enum class FollowState { SENDING, SETTLING, REACHED, STALLED, BLOCKED };

// Sends one waypoint per tick and judges from the joint readings whether the arm keeps up. No ROS.
class PathFollower {
public:
    explicit PathFollower(const FollowSettings &settings) : settings_(settings) {}

    // Cuts the joint-space lines between corners into waypoints; the first corner is where the arm already is.
    void load(const std::vector<kine::JointAngles> &corners);

    // One joint reading. The follower ticks faster than the driver publishes, so only a reading
    // it has not seen before is allowed to decide whether a joint is keeping up.
    void measure(const kine::JointAngles &joints);
    void loseMeasurement();

    // `send` is set when `target` holds a waypoint to send this tick.
    FollowState tick(double now_s, kine::JointAngles &target, bool &send);

    size_t sent() const { return next_; }
    size_t waypointCount() const { return waypoints_.size(); }
    int    blockedJoint() const { return blocked_joint_; }

private:
    bool arrived() const;
    int  jointNotFollowing() const;
    int  jointNotClosingIn() const;
    bool strike(int joint);

    FollowSettings                 settings_;
    std::vector<kine::JointAngles> waypoints_;
    size_t                         next_          = 0;
    FollowState                    state_         = FollowState::REACHED;
    double                         settle_start_s_ = 0.0;

    kine::JointAngles now_{};
    kine::JointAngles previous_{};
    bool              have_now_      = false;
    bool              have_previous_ = false;
    bool              unjudged_      = false;  // a reading arrived since the last tick that judged one
    int               strikes_       = 0;
    int               blocked_joint_ = -1;
};

}  // namespace executor

#endif  // EXECUTOR_PATHFOLLOWER_H
