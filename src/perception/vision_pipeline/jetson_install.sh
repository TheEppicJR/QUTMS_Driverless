source "/home/developer/mambaforge/bin/activate"
conda activate driverless_env

# install pytorch
# https://github.com/pytorch/pytorch#from-source
git clone --recursive --branch v1.10.1 https://github.com/pytorch/pytorch
cd pytorch
mamba install -y \
    astunparse \
    numpy \
    ninja \
    pyyaml \
    mkl \
    mkl-include \
    setuptools \
    cmake \
    cffi \
    typing_extensions \
    future \
    six \
    requests \
    dataclasses
export CMAKE_PREFIX_PATH=${CONDA_PREFIX:-"$(dirname $(which conda))/../"}
python3 setup.py install

# install torch vision
git clone --branch v0.11.2 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.11.2
export LD_LIBRARY_PATH=/usr/local/cuda/lib64 ${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
python3 setup.py install

# REMEMBER to update deps in conda_requirements as well
# (these deps are the same as conda_requirements with the exception of pytorch and torchvision)
mamba install -y \
    opencv==4.5.2 \
    numpy==1.21.4 \
    matplotlib==3.5.1 \
    requests==2.26.0 \
    tqdm>=4.41.0 \
    pandas>=1.1.4 \
    seaborn>=0.11.0
