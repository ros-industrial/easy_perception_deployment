# Message Meta Data
std_msgs/Header header
# Array of LocalizedObject objects. See LocalizedObject.msg for further details.
LocalizedObject[] objects
# Scene PointCloud2 information
uint32 frame_width
uint32 frame_height
sensor_msgs/PointCloud2 scene_pcl
# Time taken to process image in milliseconds
uint32 process_time
