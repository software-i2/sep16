// Copyright by BeeX [2026]

#include <bx_msgs/RosBindings.hpp>
#include <executor/ExecutorNode.h>

DECLARE_ROS_NODE_HANDLE

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    INIT_ROS_NODE("executor", 0, "executor/alive")

    params::Params executor_params("/executor");
    params::Params arm_params("/arm");
    params::Params topics_params("/topics");
    const executor::ExecutorConfig config = executor::loadExecutorConfig(executor_params, arm_params, topics_params);

    const std::string problems =
            executor_params.errors() + executor_params.unreadKeys() + arm_params.errors() + topics_params.errors();
    if (!problems.empty()) {
        LOG_ERROR("[executor] configuration is not usable:\n%s", problems.c_str());
        return 1;
    }

    executor::ExecutorNode node(config);
    LOG_INFO("[executor] ready on %s", config.action_execute.c_str());

    ROS_ASYNC_SPIN(2)
    ROS_WAIT_FOR_SHUTDOWN()
    return 0;
}
