cd weights

FILE="e2e_faster_rcnn_R_50_FPN_1x.pth"
if [ ! -f "$FILE" ]; then
    echo "Downloading FasterRCNN pretrained weights."
    wget https://download.pytorch.org/models/maskrcnn/e2e_faster_rcnn_R_50_FPN_1x.pth
fi

FILE="e2e_mask_rcnn_R_50_FPN_1x.pth"
if [ ! -f "$FILE" ]; then
    echo "Downloading MaskRCNN pretrained weights."
    wget https://download.pytorch.org/models/maskrcnn/e2e_mask_rcnn_R_50_FPN_1x.pth
fi

echo "All pretrained weights downloaded."

echo "Trimming MaskRCNN pretrained weights"

eval "$(conda shell.bash hook)"
conda activate mask_train
python trim_mask_rcnn.py
python trim_faster_rcnn.py
