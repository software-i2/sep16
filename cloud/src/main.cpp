// Copyright by BeeX [2026]

#include <bx_msgs/RosBindings.hpp>
#include <cloud/CloudNode.h>

#include <memory>
#include <stdexcept>

DECLARE_ROS_NODE_HANDLE

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    INIT_ROS_NODE("cloud", 0, "cloud/alive")

    params::Params cloud_params("/cloud");
    params::Params camera_params("/camera");
    params::Params arm_params("/arm");
    params::Params jaws_params("/jaws");
    params::Params topics_params("/topics");
    const cloud::CloudConfig config =
            cloud::loadCloudConfig(cloud_params, camera_params, arm_params, jaws_params, topics_params);

    const std::string problems = cloud_params.errors() + cloud_params.unreadKeys() + camera_params.errors()
                                 + arm_params.errors() + jaws_params.errors() + topics_params.errors();
    if (!problems.empty()) {
        LOG_ERROR("[cloud] configuration is not usable:\n%s", problems.c_str());
        return 1;
    }

    std::unique_ptr<cloud::CloudPipeline> pipeline;
    try {
        pipeline.reset(new cloud::CloudPipeline(config));
    } catch (const std::invalid_argument &e) {
        LOG_ERROR("[cloud] %s", e.what());
        return 1;
    }

    cloud::CloudNode node(config, *pipeline);
    LOG_INFO("[cloud] ready on %s: crop radius %.3f m (reach %.3f + margin %.3f), voxels of %.1f mm",
             config.action_collect.c_str(), config.crop_radius, config.candidate_reach, config.crop_margin,
             config.voxel_size * 1000.0);

    ROS_ASYNC_SPIN(2)
    ROS_WAIT_FOR_SHUTDOWN()
    return 0;
}
