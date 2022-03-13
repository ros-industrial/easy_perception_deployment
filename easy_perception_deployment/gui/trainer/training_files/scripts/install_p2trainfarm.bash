#!/usr/bin/env bash

export START_DIR=$PWD

path_to_dataset=$1
path_to_modif=$2
path_to_config=$3
path_to_trim_tool=$4

if [ ! -d "trainer/P2TrainFarm" ]; then
  mkdir -p trainer/P2TrainFarm && cd trainer/P2TrainFarm
  mkdir trained_models
fi

cd trainer/P2TrainFarm

export FINAL_DIR=$PWD

# Checking if the p2_trainer conda environment has been installed.
env_exists=$(conda env list | grep p2_trainer)

if [ -z "$env_exists" ]
then
      echo "Installing p2_trainer conda environment."
      conda create --name p2_trainer python=3.6.9 -y
      eval "$(conda shell.bash hook)"
      conda activate p2_trainer
      conda install ipython pip -y
      pip3 install ninja yacs cython matplotlib tqdm opencv-python

      conda install -c pytorch pytorch-nightly cudatoolkit=10.0 -y
      conda install pytorch==1.2.0 torchvision -y # Temp for magic fix.

      # Download maskrcnn-benchmark. Await modification later before building.
      git clone https://github.com/facebookresearch/maskrcnn-benchmark.git
      cd maskrcnn-benchmark
      export INSTALL_DIR=$PWD

      # Install pycocotools
      git clone https://github.com/cardboardcode/cocoapi.git
      cd cocoapi/PythonAPI
      python3 setup.py build_ext install

      # Install cityscapeScripts
      cd $INSTALL_DIR
      git clone https://github.com/cardboardcode/cityscapesScripts.git
      cd cityscapesScripts/
      python3 setup.py build_ext install

      # Install apex
      cd $INSTALL_DIR
      git clone https://github.com/cardboardcode/apex.git
      cd apex
      python3 setup.py install

      # Modify maskrcnn and then building it.
      echo "Modifying maskrcnn-benchmark and then building it."
      cd $START_DIR
      cp $path_to_modif $INSTALL_DIR/maskrcnn_benchmark/config/paths_catalog.py

      # Change maskrcnn_benchmark/config/paths_catalog.py
      # Insert this modification under DATASETS
      # "coco_custom_train": {
      #           "img_dir": "custom_dataset/train_dataset/",
      #           "ann_file": "custom_dataset/train_dataset/annotations.json"
      #       },
      #       "coco_custom_val": {
      #           "img_dir": "custom_dataset/val_dataset/",
      #           "ann_file": "custom_dataset/val_dataset/annotations.json"
      #       },
      cd $INSTALL_DIR
      python3 setup.py build install

      cd $INSTALL_DIR
      mkdir -p configs/custom
      # cd $START_DIR
      # cp $path_to_config $INSTALL_DIR/configs/custom/fasterrcnn_training.yaml

      cd $INSTALL_DIR
      mkdir weights && cd weights
      FILE="e2e_faster_rcnn_R_50_FPN_1x.pth"
      if [ ! -f "$FILE" ]; then
          echo "Downloading FasterRCNN pretrained weights."
          wget https://download.pytorch.org/models/maskrcnn/e2e_faster_rcnn_R_50_FPN_1x.pth
      fi
      unset FILE
      cd $START_DIR
      cp $path_to_trim_tool $INSTALL_DIR/weights/trim_faster_rcnn.py
      cd $INSTALL_DIR/weights
      python3 trim_faster_rcnn.py

      cd $INSTALL_DIR
      mkdir datasets

      echo "[p2_trainer] conda env created."
else
      echo "[p2_trainer] - FOUND. Skipping installation."
fi

# cd $START_DIR
# echo "Transferring custom dataset to $INSTALL_DIR/datasets"
# cp -r $path_to_dataset  $INSTALL_DIR/datasets/custom_dataset

echo "TrainFarm created under $FINAL_DIR"
unset env_exists INSTALL_DIR FINAL_DIR START_DIR
