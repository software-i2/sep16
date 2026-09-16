// Copyright by BeeX [2026]

#ifndef PLANNER_CANDIDATEPLANNER_H
#define PLANNER_CANDIDATEPLANNER_H

#include <kine/ArmBody.h>
#include <msgs/ObstacleMap.h>
#include <planner/GraspGoals.h>
#include <planner/JointSpace.h>
#include <planner/PlannerConfig.h>

#include <functional>
#include <string>
#include <vector>

namespace planner {

struct PlanOutcome {
    bool                 success      = false;
    std::string          summary;
    size_t               chosen_index = 0;
    JointPath            path;  // model radians, start first
    double               cost        = 0.0;
    int                  goals_tried = 0;
    std::vector<Outcome> outcomes;  // one per candidate
};

// Finds every allowed grasp, plans paths to the cheapest-looking ones, and keeps the cheapest path.
class CandidatePlanner {
public:
    explicit CandidatePlanner(const PlannerConfig &config);

    // `start` in model radians. `cancelled` is polled between grasps.
    PlanOutcome plan(const std::vector<Candidate> &candidates, const msgs::ObstacleMap &map,
                     const kine::JointAngles &start, const std::function<bool()> &cancelled) const;

    const kine::ArmModel &model() const { return body_.model(); }

private:
    bool snapIntoLimits(const kine::JointAngles &reading, kine::JointAngles &out) const;

    PlannerConfig config_;
    kine::ArmBody body_;
};

}  // namespace planner

#endif  // PLANNER_CANDIDATEPLANNER_H
