# 3d_detection
cj 프로젝트 3d detection 

# 📑 Overview
- [Introduction](#-introduction)  
- [Key Features](#-key-features)  
- [Used Sensor](#-used-sensor)  
- [Pineline](#-pineline)  
&nbsp;&nbsp;&nbsp;[1. 2D Detection (DINO)](#1-2d-detection-dino)  
&nbsp;&nbsp;&nbsp;[2. Tracking (OC-SORT)](#2-tracking-oc-sort)  
&nbsp;&nbsp;&nbsp;[3. 3D Detection (Ultratics ROS)](#3-3d-detection-ultratics-ros)  
- [Results](#-results)  
- [Getting Started](#-getting-started)  
&nbsp;&nbsp;&nbsp;[1. Installation](#1-installation)  
&nbsp;&nbsp;&nbsp;[2. How to use](#2-how-to-use)  
&nbsp;&nbsp;&nbsp;[3. Docker](#3-docker)  
- [References](#-references)
# 🚀 Introduction

# 🔑 Key Features

# 🤖 Used Sensor

# 🛠 Pineline

## 1. 2D Detection (DINO)

## 2. Tracking (OC-SORT)

## 3. 3D Detection (Ultratics ROS)

# 📊 Results

# 🔧 Getting Started

## 1. Installation

## 1. RTX 30 Series
**test : Python=3.8, PyTorch=1.12.1, Torchvision=0.13.1, CUDA=11.6**
<br>
### i. Setting
```
pip install torch==1.12.1+cu116 torchvision==0.13.1+cu116 \
  --extra-index-url https://download.pytorch.org/whl/cu116
```
<br>

### ii. Build package
```
git clone https://github.com/happious/3d_detection
cd 3d_detection
pip install -r requirements.txt
```

```
cd src/ultralytics_ros/DINO
mkdir weights
mv ~/Downloads/checkpoint0029_4scale_swin.pth ~/3d_detection/src/ultralytics_ros/DINO/weights/
```
<br>

### iii. Catkin make
```
cd ~/3d_detection
catkin_make
source devel/setup.bash
```

<br><br>


## 2. How to use
```
roslaunch ultralytics_ros tracking.launch
```
<br>

```
roslaunch ultralytics_ros tracker_with_cloud_ros1.launch
```

## 3. Docker

# 📕 References

