#!/usr/bin/env bash

cd /home/user/p3_exporter/

# Copy over custom_dataset
if [ ! -f "/home/user/trained.pth" ] ; then
  	echo "[ERROR] - [ trained.pth ] NOT FOUND..."
else
	echo "[ trained.pth ] FOUND. Transferring to p3_exporter/weights/custom/trained.pth"
	cp --force -r /home/user/trained.pth weights/custom/
fi

# Copy over maskrcnn_training.yaml
echo "Transferring maskrcnn_export.yaml to p3_exporter/configs/custom/maskrcnn_export.yaml"
cp --force /home/user/trainer/exporter_files/maskrcnn_export.yaml configs/custom/maskrcnn_export.yaml
