// Copyright by BeeX [2026]

#include <bx_msgs/RosBindings.hpp>
#include <task/TaskNode.h>

DECLARE_ROS_NODE_HANDLE

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    INIT_ROS_NODE("task", 0, "task/alive")

    params::Params    task_params("/task");
    params::Params    arm_params("/arm");
    params::Params    topics_params("/topics");
    const task::TaskConfig config = task::loadTaskConfig(task_params, arm_params, topics_params);

    const std::string problems =
            task_params.errors() + task_params.unreadKeys() + arm_params.errors() + topics_params.errors();
    if (!problems.empty()) {
        LOG_ERROR("[task] configuration is not usable:\n%s", problems.c_str());
        return 1;
    }

    ROS_ASYNC_SPIN(2)
    task::TaskNode node(config);

    ros::Rate rate(config.rate_hz);
    while (IS_ROS_NODE_OK()) {
        node.tick();
        rate.sleep();
    }
    ROS_SHUTDOWN();
    return 0;
}
