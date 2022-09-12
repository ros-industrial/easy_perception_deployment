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

0.2.2 (2022-08-09)
-------------------
* Modified CMake-based check for CUDA installation to set USE_GPU flag. 
* Included python script within scripts folder to allow for session and use-case configuration via commandline/terminal.
* Included unit-testing as well as additional Continuous Integration component as CLI_CI.
* Contributor(s): Bey Hao Yun

0.2.3 (2022-08-12)
-------------------
* Included extant EPD GUI unit-testing as well as additional Continuous Integration component as GUI_CI.
* Improved yaml parser in P2Trainer.py and P3Trainer.py.
* Contributor(s): Bey Hao Yun


0.3.0 (2022-08-19)
-------------------
* Integrated dockerized workflow for P3TrainFarm and P3Exporter. 
* Integrated dockerized workflow for P2TrainFarm and P2Exporter.
* Deprecated Precision Level 1 training and deployment features due to lack of reported use. This removal will help reduce software bloat and accelerate GUI deployment.
* Added GUI feature to edit MAX_ITER, TEST_PERIOD, CHECKPOINT_PERIOD, STEPS parameters for maskrcnn_training, fasterrcnn_training yaml files. This allows condensed testing as well as higher customizability in training.
* Included CLI feature to edit input_image_topic.
* Revamped GUI icons for better UI appearance.
* Removed deprecated gui.png. Desktop icon is replaced with epd_desktop.png.
* Removed "Use Case =" label in Deploy GUI window. Lengthened Visualize/Action toggle for neater UI appearance.
* Added custom_dataset image collation with test label_list for GPU local reproducible unit-testing.
* Contributor(s): Bey Hao Yun

0.3.1 (2022-08-20)
------------------
* Added utilty bash script to allow users to purge all trainer/exporter related files, folders, docker images and containers.
* Fix for potential faulty default p3_train_verification.json and p2_train_verification.json.
* Fix for confusing GUI Training terminal misprints that occur whenever validating datasets.
* Elaborated checks for COCO-formatted training datasets when using Generate Dataset GUI feature.
* Spilt GUI train button into train and export buttons for more modular operations and thus easier debugging.
* Implemented logging system for GUI.
* Shifted Validate Dataset feature to run automatically once Choose Dataset has been called.
* Renamed Validate Dataset to Validate Training.
* Combined initModel to setModel in Train.py GUI Window features.
* Removed overlooked debug statements in Counting GUI Window.
* Modified Counting GUI window to close when writeToUseCaseConfig function is called.
* Contributor(s): Bey Hao Yun

0.3.2 (2022-09-05)
-------------------
* Included FORCE_CUDA variable when creating P2 and P3 training & exporting docker containers. Verified with fresh install. 
* Contributor(s): Bey Hao Yun

0.3.3 (2022-09-14)
-------------------
* Fix for [#15](https://github.com/cardboardcode/easy_perception_deployment/issues/15).  
* Contributor(s): Bey Hao Yun