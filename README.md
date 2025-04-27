# Unitree Go1 Deployment

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
