// Copyright by BeeX [2026]

#include <bx_msgs/RosBindings.hpp>
#include <planner/PlannerNode.h>

#include <memory>
#include <stdexcept>

DECLARE_ROS_NODE_HANDLE

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    INIT_ROS_NODE("planner", 0, "planner/alive")

    params::Params planner_params("/planner");
    params::Params arm_params("/arm");
    params::Params jaws_params("/jaws");
    params::Params topics_params("/topics");
    const planner::PlannerConfig config =
            planner::loadPlannerConfig(planner_params, arm_params, jaws_params, topics_params);

    const std::string problems = planner_params.errors() + planner_params.unreadKeys() + arm_params.errors()
                                 + jaws_params.errors() + topics_params.errors();
    if (!problems.empty()) {
        LOG_ERROR("[planner] configuration is not usable:\n%s", problems.c_str());
        return 1;
    }

    std::unique_ptr<planner::PlannerNode> node;
    try {
        node.reset(new planner::PlannerNode(config));
    } catch (const std::invalid_argument &e) {
        LOG_ERROR("[planner] the arm configuration is not usable: %s", e.what());
        return 1;
    }
    LOG_INFO("[planner] ready on %s", config.action_plan.c_str());

    ROS_ASYNC_SPIN(2)
    ROS_WAIT_FOR_SHUTDOWN()
    return 0;
}
