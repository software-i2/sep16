// Copyright by BeeX [2026]

#include <executor/PathFollower.h>

#include <algorithm>
#include <cmath>

namespace executor {

void PathFollower::load(const std::vector<kine::JointAngles> &corners) {
    waypoints_.clear();
    for (size_t k = 1; k < corners.size(); ++k) {
        double largest = 0.0;
        for (int j = 0; j < kine::JOINT_COUNT; ++j) {
            largest = std::max(largest, std::fabs(corners[k][j] - corners[k - 1][j]));
        }
        const int steps = std::max(1, static_cast<int>(std::ceil(largest / settings_.max_joint_step)));
        for (int s = 1; s <= steps; ++s) {
            kine::JointAngles waypoint;
            for (int j = 0; j < kine::JOINT_COUNT; ++j) {
                waypoint[j] = corners[k - 1][j] + (corners[k][j] - corners[k - 1][j]) * s / steps;
            }
            waypoints_.push_back(waypoint);
        }
    }
    next_          = 0;
    strikes_       = 0;
    blocked_joint_ = -1;
    have_previous_ = false;
    state_         = waypoints_.empty() ? FollowState::REACHED : FollowState::SENDING;
}

void PathFollower::measure(const kine::JointAngles &joints) {
    now_      = joints;
    have_now_ = true;
}

// A stale reading must not count as arrival or as a joint that stopped.
void PathFollower::loseMeasurement() {
    have_now_      = false;
    have_previous_ = false;
    strikes_       = 0;
}

FollowState PathFollower::tick(double now_s, kine::JointAngles &target, bool &send) {
    send = false;
    if (state_ == FollowState::SENDING) {
        if (strike(jointNotFollowing())) {
            return state_;
        }
        target = waypoints_[next_++];
        send   = true;
        if (next_ >= waypoints_.size()) {
            settle_start_s_ = now_s;
            state_          = FollowState::SETTLING;
        }
        return state_;
    }

    if (state_ != FollowState::SETTLING) {
        return state_;
    }
    if (arrived()) {
        state_ = FollowState::REACHED;
    } else if (!strike(jointNotClosingIn()) && now_s - settle_start_s_ > settings_.arrival_timeout_s) {
        state_ = FollowState::STALLED;
    }
    return state_;
}

bool PathFollower::arrived() const {
    if (!have_now_) {
        return false;
    }
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        if (std::fabs(now_[j] - waypoints_.back()[j]) > settings_.arrival_tolerance) {
            return false;
        }
    }
    return true;
}

// While sending: a joint asked to move that barely moved since the last tick.
int PathFollower::jointNotFollowing() const {
    if (!have_now_ || !have_previous_ || next_ < 2) {
        return -1;
    }
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        const double asked = std::fabs(waypoints_[next_ - 1][j] - waypoints_[next_ - 2][j]);
        if (asked < settings_.blocked_min_commanded_step) {
            continue;
        }
        if (std::fabs(now_[j] - previous_[j]) < settings_.blocked_min_follow_fraction * asked) {
            return j;
        }
    }
    return -1;
}

// While settling: a joint still far from the goal that barely closed in since the last tick.
int PathFollower::jointNotClosingIn() const {
    if (!have_now_ || !have_previous_) {
        return -1;
    }
    const kine::JointAngles &goal = waypoints_.back();
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        const double gap_before = std::fabs(goal[j] - previous_[j]);
        const double owed = std::min(gap_before - settings_.arrival_tolerance, settings_.max_joint_step);
        if (owed < settings_.blocked_min_commanded_step) {
            continue;
        }
        const double closed = gap_before - std::fabs(goal[j] - now_[j]);
        if (closed < settings_.blocked_min_follow_fraction * owed) {
            return j;
        }
    }
    return -1;
}

// Counts consecutive ticks with a joint not following; enough of them means the arm hit something.
bool PathFollower::strike(int joint) {
    if (joint >= 0 && ++strikes_ >= settings_.blocked_strikes) {
        blocked_joint_ = joint;
        state_         = FollowState::BLOCKED;
        return true;
    }
    if (joint < 0) {
        strikes_ = 0;
    }
    previous_      = now_;
    have_previous_ = have_now_;
    return false;
}

}  // namespace executor
