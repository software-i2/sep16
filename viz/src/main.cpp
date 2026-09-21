// Copyright by BeeX [2026]

#include <bx_msgs/RosBindings.hpp>
#include <viz/VizNode.h>

#include <memory>
#include <stdexcept>

DECLARE_ROS_NODE_HANDLE

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    INIT_ROS_NODE("viz", 0, "viz/alive")

    params::Params viz_params("/viz");
    params::Params arm_params("/arm");
    params::Params jaws_params("/jaws");
    params::Params planner_params("/planner");
    params::Params topics_params("/topics");
    const viz::VizConfig config =
            viz::loadVizConfig(viz_params, arm_params, jaws_params, planner_params, topics_params);

    const std::string problems = viz_params.errors() + viz_params.unreadKeys() + arm_params.errors()
                                 + jaws_params.errors() + planner_params.errors() + topics_params.errors();
    if (!problems.empty()) {
        LOG_ERROR("[viz] configuration is not usable:\n%s", problems.c_str());
        return 1;
    }

    std::unique_ptr<viz::VizNode> node;
    try {
        node.reset(new viz::VizNode(config));
    } catch (const std::invalid_argument &e) {
        LOG_ERROR("[viz] the arm configuration is not usable: %s", e.what());
        return 1;
    }
    LOG_INFO("[viz] drawing in %s", config.base_frame.c_str());

    ROS_ASYNC_SPIN(1)
    ros::Rate rate(config.arm_body_rate_hz);
    while (IS_ROS_NODE_OK()) {
        node->drawArmBody();
        node->drawFloor();
        rate.sleep();
    }
    ROS_SHUTDOWN();
    return 0;
}
