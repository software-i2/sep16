# Occupied voxels. Cell index = x + size_x * (y + size_y * z); origin is the outer corner of cell 0.
std_msgs/Header header
float64 voxel_size_m
geometry_msgs/Point origin
uint32 size_x
uint32 size_y
uint32 size_z

# Nothing on the arm may come near these.
uint32[] obstacle_cells

# Handle surface and approach corridors: the arm keeps clear, the jaw blades may come close but not enter.
uint32[] handle_cells
