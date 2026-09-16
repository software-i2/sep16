std_msgs/Header header
bool success
string summary

GraspPose chosen
uint32 chosen_index
JointPath path

# Weighted joint travel along the path, radians.
float64 cost

# One per candidate: ok, not_planned, no_path, obstacle, floor, no_jaw_roll, off_approach, joint_limit or unreachable.
string[] candidate_outcomes
