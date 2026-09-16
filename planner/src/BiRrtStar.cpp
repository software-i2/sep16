// Copyright by BeeX [2026]

#include <planner/BiRrtStar.h>

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

BiRrtStar::BiRrtStar(CollisionChecker &checker, const RrtSettings &settings, const kine::JointAngles &weights)
        : checker_(checker), settings_(settings), weights_(weights) {}

bool BiRrtStar::plan(const kine::JointAngles &start, const kine::JointAngles &goal, JointPath &corners) {
    random_.seed(static_cast<uint64_t>(settings_.random_seed));
    corners.clear();

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

    std::vector<std::pair<int, int>> joins;  // node in from_start, node in from_goal
    int                              first_join_iteration = -1;
    const Clock::time_point          started              = Clock::now();

    for (int iteration = 0; iteration < settings_.max_iterations; ++iteration) {
        if (secondsSince(started) > settings_.time_budget_s) {
            break;
        }
        if (first_join_iteration >= 0 && iteration - first_join_iteration >= settings_.refine_iterations) {
            break;
        }

        const int added = extend(*growing, randomJoints());
        if (added >= 0) {
            const int reached = connect(*other, (*growing)[added].joints);
            if (reached >= 0) {
                joins.push_back(growing == &from_start ? std::make_pair(added, reached)
                                                       : std::make_pair(reached, added));
                if (first_join_iteration < 0) {
                    first_join_iteration = iteration;
                }
            }
        }
        std::swap(growing, other);
    }

    if (joins.empty()) {
        return false;
    }

    size_t best = 0;
    for (size_t k = 1; k < joins.size(); ++k) {
        const double cost      = from_start[joins[k].first].cost + from_goal[joins[k].second].cost;
        const double best_cost = from_start[joins[best].first].cost + from_goal[joins[best].second].cost;
        if (cost < best_cost) {
            best = k;
        }
    }

    for (int node = joins[best].first; node >= 0; node = from_start[node].parent) {
        corners.push_back(from_start[node].joints);
    }
    std::reverse(corners.begin(), corners.end());
    for (int node = from_goal[joins[best].second].parent; node >= 0; node = from_goal[node].parent) {
        corners.push_back(from_goal[node].joints);
    }

    shortcut(corners);
    return true;
}

// Checks the inside of the edge coarse to fine, so a blocked edge is usually found early. The ends are not checked.
bool BiRrtStar::edgeClear(const kine::JointAngles &a, const kine::JointAngles &b) {
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

kine::JointAngles BiRrtStar::randomJoints() {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const kine::ArmModel                  &model = checker_.model();
    kine::JointAngles                      joints;
    for (int j = 0; j < kine::JOINT_COUNT; ++j) {
        joints[j] = model.lowerLimit(j) + (model.upperLimit(j) - model.lowerLimit(j)) * unit(random_);
    }
    return joints;
}

int BiRrtStar::nearest(const Tree &tree, const kine::JointAngles &target, double &distance) const {
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

// Adds one node towards `target`, attached to the cheapest clear neighbour, and rewires the neighbours through it.
int BiRrtStar::extend(Tree &tree, const kine::JointAngles &target) {
    double    distance = 0.0;
    const int closest  = nearest(tree, target, distance);
    if (distance < kSameJoints) {
        return -1;
    }

    const kine::JointAngles joints = distance <= settings_.extend_step
                                             ? target
                                             : interpolate(tree[closest].joints, target, settings_.extend_step / distance);
    if (checker_.check(joints) != Verdict::CLEAR) {
        return -1;
    }

    const double node_count = static_cast<double>(tree.size() + 1);
    const double radius     = std::min(settings_.extend_step,
                                       settings_.rewire_gamma
                                               * std::pow(std::log(node_count) / node_count, 1.0 / kine::JOINT_COUNT));

    std::vector<std::pair<double, int>> neighbours;  // cost through the neighbour, neighbour
    for (size_t i = 0; i < tree.size(); ++i) {
        const double d = jointDistance(weights_, tree[i].joints, joints);
        if (d <= radius || static_cast<int>(i) == closest) {
            neighbours.emplace_back(tree[i].cost + d, static_cast<int>(i));
        }
    }
    std::sort(neighbours.begin(), neighbours.end());

    int parent = -1;
    for (const std::pair<double, int> &neighbour : neighbours) {
        if (edgeClear(tree[neighbour.second].joints, joints)) {
            parent = neighbour.second;
            break;
        }
    }
    if (parent < 0) {
        return -1;
    }

    const int added = static_cast<int>(tree.size());
    tree.emplace_back();
    tree[added].joints = joints;
    tree[added].parent = parent;
    tree[added].cost   = tree[parent].cost + jointDistance(weights_, tree[parent].joints, joints);
    tree[parent].children.push_back(added);

    for (const std::pair<double, int> &neighbour : neighbours) {
        const int node = neighbour.second;
        if (node == parent) {
            continue;
        }
        const double through_added = tree[added].cost + jointDistance(weights_, joints, tree[node].joints);
        if (through_added + kSameJoints < tree[node].cost && edgeClear(joints, tree[node].joints)) {
            reparent(tree, node, added, through_added);
        }
    }
    return added;
}

// Extends repeatedly towards `target` until it is reached or blocked.
int BiRrtStar::connect(Tree &tree, const kine::JointAngles &target) {
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

void BiRrtStar::reparent(Tree &tree, int node, int new_parent, double new_cost) {
    std::vector<int> &siblings = tree[tree[node].parent].children;
    siblings.erase(std::remove(siblings.begin(), siblings.end(), node), siblings.end());
    tree[node].parent = new_parent;
    tree[new_parent].children.push_back(node);

    const double     change = new_cost - tree[node].cost;
    std::vector<int> pending(1, node);
    while (!pending.empty()) {
        const int current = pending.back();
        pending.pop_back();
        tree[current].cost += change;
        pending.insert(pending.end(), tree[current].children.begin(), tree[current].children.end());
    }
}

void BiRrtStar::shortcut(JointPath &corners) {
    for (int attempt = 0; attempt < settings_.shortcut_attempts && corners.size() > 2; ++attempt) {
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
