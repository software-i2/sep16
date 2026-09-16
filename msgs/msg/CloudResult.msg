# What the cloud node made of the collected frames, in the arm base frame.
std_msgs/Header header
bool success
string summary
uint32 frames_used

# Every grasp pose of the newest frame, split by the handle classifier.
GraspPose[] handle_poses
GraspPose[] rope_poses

# Handle poses within reach, averaged over frames when that is switched on.
GraspPose[] candidates

ObstacleMap obstacles
