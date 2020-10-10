# Take in input from training phase.

# To switch to onnx_stage branch run git fetch origin onnx_stage
git clone git@github.com:BowenBao/maskrcnn-benchmark.git --branch onnx_stage_mrcnn
cd maskrcnn-benchmark
eval "$(conda shell.bash hook)"
conda create --name p3_onnx_exporter python=3.6
conda activate p3_onnx_exporter
pip install ipython ninja yacs cython matplotlib tqdm opencv-python requests onnx onnxruntime

conda install -c pytorch pytorch-nightly cudatoolkit=10.0 -y
conda install pytorch==1.2.0 torchvision -y # Temp for magic fix.
python setup.py build install

echo "This script installs pycocotools and apex."
INSTALL_DIR=$PWD
# install pycocotools
echo "Installing pycocotools from source"
cd $INSTALL_DIR
git clone https://github.com/cocodataset/cocoapi.git
cd cocoapi/PythonAPI
python setup.py build_ext install
cd ../../

# install apex
cd $INSTALL_DIR
git clone https://github.com/NVIDIA/apex.git
cd apex
if output=$(python setup.py install --cuda_ext --cpp_ext > /dev/null 2>&1); then
    echo "Build failed. But don't worry. Running contingency command."
else
    python setup.py install
fi
