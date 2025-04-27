# Unitree Go1 Deployment in Jetson

## Installation

### 1. SDK(with LCM forward)

```bash
sudo apt update
sudo apt install libboost-all-dev
# install LCM
sudo apt install liblcm-dev

# complie sdk
cd unitree_legged_sdk
mkdir build
cmake ..
make
```

### 2. Deployment code

If Jetson not install conda yet:

```bash
# open a new terminal and back to home
cd ~

# 1. Download ARM64  Miniforge3 sh file
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh

# 2. assign permission
chmod +x Miniforge3-Linux-aarch64.sh

# 3. install
# Follow the installation tutorial in the terminal
./Miniforge3-Linux-aarch64.sh

# 5. Test installation
source ~/.bashrc
conda --version

```

Install pytorch first, we don't recommand using pip install torch directly, might get x86 version if you are using jetson.

Go and check https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048 and find matched version pytorch wheel for your system.

If you don't know your system jetpack, use the command `cat /etc/nv_tegra_release`. My jetpack is 5.1, 

```bash
# create environment and activate
conda create -n go1_deploy python=3.8
conda activate go1_deploy

sudo apt install libopenblas-base libopenblas-dev

# find pytorch wheel of your system and replace
wget https://developer.download.nvidia.com/compute/redist/jp/v502/pytorch/torch-1.13.0a0+d0d6b1f2.nv22.10-cp38-cp38-linux_aarch64.whl
# install
pip install torch-1.13.0a0+d0d6b1f2.nv22.10-cp38-cp38-linux_aarch64.whl

```

Install the environment
```bash
# create environment and activate
conda create -n go1_deploy python=3.8
conda activate go1_deploy

# install
cd go1_deploy
pip install -e .
```