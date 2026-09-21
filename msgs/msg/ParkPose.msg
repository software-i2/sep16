# Where the vehicle should park, as a displacement of the vehicle body in the frame it was
# asked from. Yaw is about the body z axis; the search does not tilt the vehicle.
geometry_msgs/Vector3 position
float64 yaw

# What the search expected there, so a caller can see why this pose won.
uint32  grasps_admitted        # candidates the reach table let through
uint32  grasps_total           # candidates it was offered
float64 travel                 # metres of body translation the move costs
float64 score
