// Copyright by BeeX [2026]

#include <kine/ForwardKinematics.h>
#include <kine/InverseKinematics.h>
#include <planner/GraspGoals.h>
#include <planner/JointSpace.h>

#include <algorithm>

namespace planner {
namespace {

constexpr double kShortestDirection = 1e-9;

Outcome furthest(Outcome a, Outcome b) { return static_cast<int>(b) > static_cast<int>(a) ? b : a; }

double angleBetween(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
    const double cosine = a.dot(b) / (a.norm() * b.norm());
    return std::acos(std::max(-1.0, std::min(1.0, cosine)));
}

}  // namespace

const char *outcomeName(Outcome outcome) {
    switch (outcome) {
    case Outcome::UNREACHABLE:
        return "unreachable";
    case Outcome::JOINT_LIMIT:
        return "joint_limit";
    case Outcome::OFF_APPROACH:
        return "off_approach";
    case Outcome::NO_JAW_ROLL:
        return "no_jaw_roll";
    case Outcome::FLOOR:
        return "floor";
    case Outcome::OBSTACLE:
        return "obstacle";
    case Outcome::NOT_PLANNED:
        return "not_planned";
    case Outcome::NO_PATH:
        return "no_path";
    case Outcome::OK:
        return "ok";
    }
    return "unknown";
}

Outcome findGraspGoals(const Candidate &candidate, size_t index, const kine::JointAngles &start,
                       const GraspSettings &settings, CollisionChecker &checker, std::vector<GraspGoal> &goals) {
    const kine::ArmModel &model = checker.model();
    const double          along = model.dimensions().wrist_to_jaw_mount + settings.grasp_point_from_mount;
    const bool check_approach   = settings.max_approach_deviation > 0.0 && candidate.approach.norm() > kShortestDirection;

    Outcome furthest_failure = Outcome::UNREACHABLE;
    bool    found            = false;

    for (const bool elbow_up : {false, true}) {
        kine::JointAngles    joints;
        const kine::IkResult ik = kine::solvePosition(model, candidate.point, along, elbow_up, start, joints);
        if (ik == kine::IkResult::JOINT_LIMIT) {
            furthest_failure = furthest(furthest_failure, Outcome::JOINT_LIMIT);
            continue;
        }
        if (ik != kine::IkResult::SOLVED) {
            continue;
        }

        if (check_approach
            && angleBetween(kine::jawAxes(model, joints).approach, candidate.approach) > settings.max_approach_deviation) {
            furthest_failure = furthest(furthest_failure, Outcome::OFF_APPROACH);
            continue;
        }

        double rolls[2];
        if (kine::wristRollsAcrossBar(model, joints, candidate.bar_axis, rolls) == 0) {
            furthest_failure = furthest(furthest_failure, Outcome::NO_JAW_ROLL);
            continue;
        }

        for (const double roll : rolls) {
            if (!kine::fitIntoLimits(model, kine::WRIST, roll, start[kine::WRIST], joints[kine::WRIST])) {
                furthest_failure = furthest(furthest_failure, Outcome::NO_JAW_ROLL);
                continue;
            }
            const Verdict verdict = checker.check(joints);
            if (verdict == Verdict::FLOOR) {
                furthest_failure = furthest(furthest_failure, Outcome::FLOOR);
            } else if (verdict == Verdict::OBSTACLE) {
                furthest_failure = furthest(furthest_failure, Outcome::OBSTACLE);
            } else if (verdict == Verdict::JOINT_LIMIT) {
                furthest_failure = furthest(furthest_failure, Outcome::JOINT_LIMIT);
            } else {
                goals.push_back({index, joints, jointTravel(settings.joint_cost_weights, start, joints)});
                found = true;
            }
        }
    }
    return found ? Outcome::NOT_PLANNED : furthest_failure;
}

}  // namespace planner
