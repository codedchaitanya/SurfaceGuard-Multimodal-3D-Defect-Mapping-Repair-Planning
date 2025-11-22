# msg/Defect.msg

# Standard header for timestamp and frame_id (crucial for TF2 transforms)
std_msgs/Header header

# Type of defect detected by the Deep Learning node
# Examples: "crack", "spalling", "screw_hole", "uneven_joint"
string defect_type

# The 3D center of the defect in the camera frame (x, y, z)
geometry_msgs/Point32 centroid

# Approximate surface area in square meters (calculated from point cloud cluster)
float32 surface_area

# Confidence score of the detection (0.0 to 1.0)
float32 confidence