# Message Meta Data
std_msgs/Header header
# Array of LocalizedObject objects. See LocalizedObject.msg for further details.
LocalizedObject[] objects
# Array of unique string ids given to each object tracked.
string[] object_ids
# Depth image information
uint32 frame_width
uint32 frame_height
sensor_msgs/PointCloud2 scene_pcl
# Time taken to process image in milliseconds
uint32 process_time
