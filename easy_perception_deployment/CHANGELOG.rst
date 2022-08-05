^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easy_perception_deployment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2022-07-30)
-------------------
* Updated copyright date in root LICENSE file.
* Included CHANGELOG.rst in easy_perception_deployment ROS2 package. 
* Introduced hotfix for faulty Input Image Topic for Register Topic GUI feature.
* Contributor(s): Bey Hao Yun

0.1.0 (2022-07-31)
-------------------
* Reconfigured Register Topic GUI feature to read from .json file in config folder instead of .launch.py file directly.
* Revamped session_config and use_config parser. easy_perception_deployment now reads config/usecase_config.json as well as config/session_config.json for more robust session configuration.
* Contributor(s): Bey Hao Yun

0.1.1 (2022-08-01)
-------------------
* Fixed faulty usecase_config.json reading within usecase_config.hpp.
* Updated generateCountClassNames() in usecase_config.hpp to json parsing.
* Fixed wrong if conditions for parsing incoming Use Case integer in Deploy.py.
* Contributor(s): Bey Hao Yun

0.2.0 (2022-08-05)
-------------------
* Removed all instances of infer_visualizes from P2OrtBase and P3OrtBase classes.
* Modified Visualize workflows for all Precision Levels and Use Case configuration to use infer_action.
* Renamed all instances of infer_action to infer. 
* Removed debug statement printing onnx_model_path from epd_container.cpp.
* Transferred detection results visualization from P2OrtBase and P3OrtBase to EPDContainer class.
* Generified detection results visualization functions to minimize code verbosity.
* Modified default session_config.json.
* Modified default input_image_topic.json.
* Updated unit testing modules under test folder to reflect new infer function calls for P1OrtBase, P2OrtBase and P3OrtBase classes. 
* Contributor(s): Bey Hao Yun

0.2.1 (2022-08-06)
-------------------
* Abstracted instantiation of struct LocalizedObject in usecase_config.hpp. Reduced code verbosity for EPDObjectLocalization to EPDObjectTracking conversion in Localization Visualize workflow. 
* Contributor(s): Bey Hao Yun


