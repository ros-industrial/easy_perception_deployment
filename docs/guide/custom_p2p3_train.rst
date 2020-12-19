.. _custom_train_p2p3:

Training a P2/P3 Object Detection Model
=============================================================
What makes Computer Vision useful is the fact that models can be trained to
classify user-defined objects.

In this guide, the instructions will guide you on **how to train your very own
Precision-Level 2 or Precision-Level 3 (P3) Object Detection computer vision model** which could then used to deploy as a modular ROS2 package.

Collect Training Data
+++++++++++++++++++++
A neural network model, much like a student, requires training data to know what are the intended objects it is meant to detect. In the case of Computer Vision, these training data refers to images of user-defined objects. If the user intention is to train a model to distinguish between cats and dogs, the training data ought to be photos of cats and dogs.

Therefore, please take as many as photos or images that suit the following criteria:

+------------------+----------------------------------------------------------------------------------------------------+
| Criteria No.     | Description                                                                                        |
+==================+====================================================================================================+
| 1                | A training image **can contain more than one class of object.**                                    |
+------------------+----------------------------------------------------------------------------------------------------+
| 2                | The lighting condition in the image should be similar to the physical setup of the camera.         |
+------------------+----------------------------------------------------------------------------------------------------+
| 3                | A training image should be at least of size, 224 by 224.                                           |
+------------------+----------------------------------------------------------------------------------------------------+

The recommended number of images to capture is: ``110`` in total. However, the more the merrier.

Label Training Data
+++++++++++++++++++

Arranging Data
^^^^^^^^^^^^^^

Once you have collected your training data, please arrange the images in the following file structure:

.. code-block:: bash

   your_p2p3_dataset_name_here_dataset
      |_train_dataset/
            |_<photos of all images used for training.>
      |_val_dataset/
            |_<photos of all images used for validation.>

Please the split the ``90`` for the train dataset and ``20`` for the validation dataset.

Create Custom Label List
^^^^^^^^^^^^^^^^^^^^^^^^
Once you have arranged your file like as shown above, create a ``.txt`` label list.

.. code-block:: bash

   Eg.

   __ignore__
   _background_
   object_class_1_name
   object_class_2_name
   ...

Follow the instructions below to get started labelling your dataset.

1. Double-click on ``easy_perception_deployment.desktop`` on your Desktop. This file is generated following instructions under the `Setup <./setup.html>`_ section.

A window labelled, **easy_perception_deployment**, will appear.

2. Click on button labelled, **Train**, will appear.

A window labelled, **Train**, will appear.

3. Click on button labelled, **P2** or **P3**.

You have selected Precision Level 2 or 3 training framework.

4. Click on DropDown Menu, **Set Model** and select the model you wish to run.

There is currently **the pretrained FasterRCNN model** available within **easy_perception_deployment**.

5. Click on button labelled, **Choose Label List** and select the ``.txt`` label list file you have created under `Create Custom Label List`_.

6. Click on button labelled, **Label Dataset** and select **the training dataset folder** you have created under `Arranging Data`_.

The ``labelme`` application window should appear and you can start labelling your training images with bounding boxes/segmentation masks, corresponding to their respective object class names.

7. Click on button labelled, **Generate Dataset** and select the **the training dataset folder** you have created under `Arranging Data`_.

The labelled dataset will be converted to a COCO-formatted dataset and written to directory ``data/datasets/custom_dataset`` in the package.

**[Caution]**: DO NOT rename the custom_dataset folder. It will be strictly referenced later during training.


Train Model
+++++++++++
Continuing on from the previous section, follow the steps below to get started training and generating your P2/P3 ONNX model.

1. Click on button labelled, **Choose Dataset** and select ``data/datasets/custom_dataset`` under the following directory.

.. code-block:: bash

   easy_perception_deployment/easy_perception_deployment/datasets/custom_dataset

2. Click on button labelled, **Validate Dataset** to verify if your dataset is in the correct prescribed file structure.

If the dataset is properly arranged, the button labelled, **Train** will now be highlighted.

3. Click on button labelled, **Train** to start training.

The final ``.onnx`` trained model will be written to the following directories in the package and timestamped with the date in which it was trained in the filename.

.. code-block:: bash

   # If you are training P2 ONNX model.
   easy_perception_deployment/easy_perception_deployment/gui/trainer/P2TrainFarm/trained_models/
   # If you are training P3 ONNX model.
   easy_perception_deployment/easy_perception_deployment/gui/trainer/P3TrainFarm/trained_models/
