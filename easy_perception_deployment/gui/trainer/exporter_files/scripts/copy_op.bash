#!/usr/bin/env bash

EXPORT_MASKRCNN=$1

if $EXPORT_MASKRCNN ; then
	cd /home/user/p3_exporter
else
	cd /home/user/p2_exporter
fi
# Copy over custom_dataset
if [ ! -f "/home/user/trained.pth" ] ; then
  	echo "[ERROR] - [ trained.pth ] NOT FOUND..."
else
	echo "[ trained.pth ] FOUND. Transferring to p3_exporter/weights/custom/trained.pth"
	cp --force -r /home/user/trained.pth weights/custom/
fi

if $EXPORT_MASKRCNN ; then
	# Copy over maskrcnn_export.yaml
	echo "Transferring maskrcnn_export.yaml to p3_exporter/configs/custom/maskrcnn_export.yaml"
	cp --force /home/user/trainer/exporter_files/maskrcnn_export.yaml /home/user/p3_exporter/configs/custom/maskrcnn_export.yaml
else
	# Copy over faster_export.yaml
	echo "Transferring fasterrcnn_export.yaml to p3_exporter/configs/custom/fasterrcnn_export.yaml"
	cp --force /home/user/trainer/exporter_files/fasterrcnn_export.yaml /home/user/p2_exporter/configs/custom/fasterrcnn_export.yaml
fi