# Message Meta Data
std_msgs/Header header
# Array of LocalizedObject objects. See LocalizedObject.msg for further details.
LocalizedObject[] objects
# Intrinsic camera parameters
float64 ppx
float64 ppy
float64 fx
float64 fy
# Depth image information
uint32 frame_width
uint32 frame_height
sensor_msgs/Image depth_image
# Time taken to process image in milliseconds
uint32 process_time
