# What the cloud node made of the collected frames, in the arm base frame.
std_msgs/Header header
bool success
string summary
uint32 frames_used

# True when this came from a fresh collect, the re-measurement taken from the park pose.
bool fresh

# Every grasp pose of the newest frame.
GraspPose[] poses

# Poses within reach, averaged over frames when that is switched on.
GraspPose[] candidates

ObstacleMap obstacles
