// Copyright by BeeX [2026]

#include <planner/BiRrt.h>

#include <algorithm>
#include <chrono>
#include <limits>

namespace planner {
namespace {

constexpr double kSameJoints     = 1e-9;
constexpr int    kMaxEdgeSamples = 100000;

using Clock = std::chrono::steady_clock;

double secondsSince(const Clock::time_point &start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

}  // namespace

BiRrt::BiRrt(CollisionChecker &checker, const RrtSettings &settings, const kine::JointAngles &weights)
        : checker_(checker), settings_(settings), weights_(weights) {}

bool BiRrt::plan(const kine::JointAngles &start, const kine::JointAngles &goal, JointPath &corners, double budget_s) {
    random_.seed(static_cast<uint64_t>(settings_.random_seed));
    corners.clear();
    started_ = Clock::now();
    budget_  = budget_s;

    if (edgeClear(start, goal)) {
        corners = {start, goal};
        return true;
    }

    Tree from_start(1);
    Tree from_goal(1);
    from_start[0].joints = start;
    from_goal[0].joints  = goal;

    Tree *growing = &from_start;
    Tree *other   = &from_goal;

    int joined_start = -1;
    int joined_goal  = -1;

    for (int iteration = 0; iteration < settings_.max_iterations; ++iteration) {
        if (secondsSince(started_) > std::min(settings_.time_budget_s, budget_)) {
            break;
        }

        const int added = extend(*growing, randomJoints());
        if (added >= 0) {
            const int reached = connect(*other, (*growing)[added].joints);
            if (reached >= 0) {
                joined_start = growing == &from_start ? added : reached;
                joined_goal  = growing == &from_start ? reached : added;
                break;
            }
        }
        std::swap(growing, other);
    }

    if (joined_start < 0) {
        return false;
    }

    for (int node = joined_start; node >= 0; node = from_start[node].parent) {
        corners.push_back(from_start[node].joints);
    }
    std::reverse(corners.begin(), corners.end());
    // The joined node holds the same joints on both sides, so the goal tree starts at its parent.
    for (int node = from_goal[joined_goal].parent; node >= 0; node = from_goal[node].parent) {
        corners.push_back(from_goal[node].joints);
    }

    shortcut(corners);
    return true;
}

// Checks the inside of the edge coarse to fine, so a blocked edge is usually found early. The ends are not checked.
bool BiRrt::edgeClear(const kine::JointAngles &a, const kine::JointAngles &b) {
    double largest_step = 0.0;
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        largest_step = std::max(largest_step, std::fabs(b[j] - a[j]));
    }
    const int samples = std::min(kMaxEdgeSamples,
                                 std::max(1, static_cast<int>(std::ceil(largest_step / settings_.edge_check_step))));

    int stride = 1;
    while (stride * 2 < samples) {
        stride *= 2;
    }
    for (; stride >= 1; stride /= 2) {
        for (int i = stride; i < samples; i += 2 * stride) {
            if (checker_.check(interpolate(a, b, static_cast<double>(i) / samples)) != Verdict::CLEAR) {
                return false;
            }
        }
    }
    return true;
}

kine::JointAngles BiRrt::randomJoints() {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const kine::ArmModel                  &model = checker_.model();
    kine::JointAngles                      joints;
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        joints[j] = model.lowerLimit(j) + (model.upperLimit(j) - model.lowerLimit(j)) * unit(random_);
    }
    return joints;
}

int BiRrt::nearest(const Tree &tree, const kine::JointAngles &target, double &distance) const {
    int closest = 0;
    distance    = std::numeric_limits<double>::max();
    for (size_t i = 0; i < tree.size(); ++i) {
        const double d = jointDistance(weights_, tree[i].joints, target);
        if (d < distance) {
            distance = d;
            closest  = static_cast<int>(i);
        }
    }
    return closest;
}

// Adds one node towards `target`, attached to the nearest node already in the tree.
int BiRrt::extend(Tree &tree, const kine::JointAngles &target) {
    double    distance = 0.0;
    const int closest  = nearest(tree, target, distance);
    if (distance < kSameJoints) {
        return -1;
    }

    const kine::JointAngles joints = distance <= settings_.extend_step
                                             ? target
                                             : interpolate(tree[closest].joints, target, settings_.extend_step / distance);
    if (checker_.check(joints) != Verdict::CLEAR || !edgeClear(tree[closest].joints, joints)) {
        return -1;
    }

    const int added = static_cast<int>(tree.size());
    tree.emplace_back();
    tree[added].joints = joints;
    tree[added].parent = closest;
    return added;
}

// Extends repeatedly towards `target` until it is reached or blocked.
int BiRrt::connect(Tree &tree, const kine::JointAngles &target) {
    for (;;) {
        double    distance = 0.0;
        const int closest  = nearest(tree, target, distance);
        if (distance < kSameJoints) {
            return closest;
        }
        if (extend(tree, target) < 0) {
            return -1;
        }
    }
}

void BiRrt::shortcut(JointPath &corners) {
    for (int attempt = 0; attempt < settings_.shortcut_attempts && corners.size() > 2; ++attempt) {
        if (secondsSince(started_) > budget_) {
            break;
        }
        std::uniform_int_distribution<size_t> pick(0, corners.size() - 1);
        size_t                                i = pick(random_);
        size_t                                k = pick(random_);
        if (i > k) {
            std::swap(i, k);
        }
        if (k - i > 1 && edgeClear(corners[i], corners[k])) {
            corners.erase(corners.begin() + static_cast<long>(i) + 1, corners.begin() + static_cast<long>(k));
        }
    }
}

}  // namespace planner
