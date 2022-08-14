^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package epd-msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2021-10-10)
------------------
* Added `Perception.srv` file. This will be utilized by EPD's Service Mode. 
* Removed the intrinsic camera parameters which will not be populated and used due to EPD Architecture overhaul.
* Replaced the `depth_img` parameter in `EPDObjectLocalization.msg` and `EPDObjectTracking.msg` with **sensor_msgs/PointCloud2 scene_pcl** to reflect PCL-variant approach.
* Contributors: Bey Hao Yun (Gary)

0.3.0 (2022-08-16)
-------------------
* Deprecated EPDImageClassification.msg due to removal of Precision Level 1 features.
* Contributor(s): Bey Hao Yun
