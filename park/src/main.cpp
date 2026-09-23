// Copyright by BeeX [2026]

#include <bx_msgs/RosBindings.hpp>
#include <park/ParkNode.h>

#include <memory>
#include <stdexcept>

DECLARE_ROS_NODE_HANDLE

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    INIT_ROS_NODE("park", 0, "park/alive")

    params::Params vehicle_params("/vehicle");
    params::Params arm_params("/arm");
    params::Params jaws_params("/jaws");
    params::Params planner_params("/planner");
    params::Params camera_params("/camera");
    params::Params topics_params("/topics");
    const park::ParkConfig config =
            park::loadParkConfig(vehicle_params, arm_params, jaws_params, planner_params, camera_params,
                                 topics_params);

    const std::string problems = vehicle_params.errors() + vehicle_params.unreadKeys() + arm_params.errors()
                                 + jaws_params.errors() + planner_params.errors() + camera_params.errors()
                                 + topics_params.errors();
    if (!problems.empty()) {
        LOG_ERROR("[park] configuration is not usable:\n%s", problems.c_str());
        return 1;
    }

    std::unique_ptr<park::ParkNode> node;
    try {
        node.reset(new park::ParkNode(config));
    } catch (const std::invalid_argument &e) {
        LOG_ERROR("[park] the arm configuration is not usable: %s", e.what());
        return 1;
    }
    LOG_INFO("[park] ready on %s", config.action_park.c_str());

    ROS_ASYNC_SPIN(2)
    ROS_WAIT_FOR_SHUTDOWN()
    return 0;
}
