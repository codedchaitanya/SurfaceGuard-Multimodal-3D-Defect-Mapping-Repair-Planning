# msg/DefectArray.msg

# Timestamp for when this batch of defects was detected
std_msgs/Header header

# A list (array) of Defect messages
surface_guard_interfaces/Defect[] defects
