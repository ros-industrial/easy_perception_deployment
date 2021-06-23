.. _custom_train_p1:

Training a P1 Image Classification Model
========================================
What makes Computer Vision useful is the fact that models can be trained to
classify user-defined objects.

In this guide, the instructions will guide you on **how to train your very own
Precision-Level 1 (P1) Image Classification computer vision model** which could
then used to deploy as a modular ROS2 package.

Collect Training Data
+++++++++++++++++++++
A neural network model, much like a student, requires training data to know what are the intended objects it is meant to detect. In the case of Computer Vision, these training data refers to images of user-defined objects. If the user intention is to train a model to distinguish between cats and dogs, the training data ought to be photos of cats and dogs.

Therefore, please take as many as photos or images that suit the following criteria:

+------------------+----------------------------------------------------------------------------------------------------+
| Criteria No.     | Description                                                                                        |
+==================+====================================================================================================+
| 1                | A training image should only show one class of object.                                             |
+------------------+----------------------------------------------------------------------------------------------------+
| 2                | The lighting condition in the image should be similar to the physical setup of the camera.         |
+------------------+----------------------------------------------------------------------------------------------------+
| 3                | A training image should be at least of size, 224 by 224.                                           |
+------------------+----------------------------------------------------------------------------------------------------+

The recommended number of images to capture is: ``30`` per object class.

.. code-block:: bash

   Eg.

   30 different photos of cats
   30 different of dogs

Label Training Data
+++++++++++++++++++

Arranging Data
^^^^^^^^^^^^^^

Among the three precision levels, labelling a P1 training image dataset is the easiest. **It is only a matter of arranging files in the following file structure:**

.. code-block:: bash

   insert_your_training_here_dataset/
      |_train
        |_name_of_object_class_1
          |_<photos of object_class_1>
        |_name_of_object_class_2
          |_<photos of object_class_2>
        |_ ...
      |_val
        |_name_of_object_class_1
          |_<photos of object_class_1>
        |_name_of_object_class_2
          |_<photos of object_class_2>
        |_ ...

.. code-block:: bash

   Eg.

   p1_catsdogs_dataset/
      |_train
        |_cat
          |_<photos of cats>
        |_dog
          |_<photos of dogs>
      |_val
        |_cat
          |_<photos of cats>
        |_dog
          |_<photos of dogs>

Create Custom Label List
^^^^^^^^^^^^^^^^^^^^^^^^
Once you have arranged your file like as shown above, create a ``.txt`` label list.

.. code-block:: bash

   Eg.

   cat
   dog

Ensure that the object names are sorted in lexicographical order.

.. note::
    P2 and P3 training datasets will adopt a slightly different label list format.


Train Model
+++++++++++

Follow the instructions below to get started training and generating your P1 ONNX model.

1. Double-click on ``easy_perception_deployment.desktop`` on your Desktop. This file is generated following instructions under the `Setup <./setup.html>`_ section.

A window labelled, **easy_perception_deployment**, will appear.

2. Click on button labelled, **Train**, will appear.

A window labelled, **Train**, will appear.

3. Click on button labelled, **P1**.

You have selected Precision Level 1 training framework.

4. Click on DropDown Menu, **Set Model** and select the model you wish to run.

There are currently **only 6 pretrained model** available in the PyTorch Model Zoo. Each has its own pros and cons.

.. code-block:: bash

   [Tips on choosing]:
   # If you wish to prioritize speed over accuracy,
   Choose [mobilenet].
   # If you wish to prioritize accuracy over speed,
   Choose [densenet].

5. Click on button labelled, **Choose Label List** and select the ``.txt`` label list file you have created under `Create Custom Label List`_.

6. Click on button labelled, **Choose Dataset** and select **the training dataset folder** you have created under `Arranging Data`_.

7. Click on button labelled, **Validate Dataset** to verify if your dataset is in the correct prescribed file structure.

If the dataset is properly arranged, the button labelled, **Train** will now be highlighted.

8. Click on button labelled, **Train** to start training.

The final ``.onnx`` trained model will be written to ``/data/model/`` directory in the package and timestamped with the date in which it was trained in the filename.
