string name # object name
sensor_msgs/RegionOfInterest roi # object 2d bounding box
sensor_msgs/Image segmented_binary_mask # object 2d segmentation mask
geometry_msgs/Point centroid  # object 3d bounding box centroid
float64 length  # object 3d bounding box length
float64 breadth	# object 3d bounding box breadth
float64 height  # object 3d bounding box height
sensor_msgs/PointCloud2 segmented_pcl # object 3d segmentation cloud
geometry_msgs/Vector3 axis # object 3d bounding box object axis
