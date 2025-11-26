# OODIS: Omnidirectional Object Detection and Tracking for Industrial Safety
<결과 gif>

## 📑 Overview
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
&nbsp;&nbsp;&nbsp;[2. How to run](#2-how-to-run)  
&nbsp;&nbsp;&nbsp;[3. Docker](#3-docker)  
- [References](#-references)
## 🚀 Introduction
<img width="677" height="913" alt="image" src="https://github.com/user-attachments/assets/2cdc362f-414c-4fc1-a6e3-54a5724ca0b0" />



OODIS는 360° 카메라와 360° LiDAR를 결합한 전방위 감지 시스템을 제공하며,
ERP 기반 2D 투영과 DINO + OC-SORT로 안정적인 탐지·추적을 수행한다.
지게차와 로봇 모두에 장착 가능하며, 실내·실외·공장 환경에서 일관된 성능을 보였다.

## 🔑 Key Features
- **360° Full Surround Perception** <br>
&nbsp;&nbsp;&nbsp;◦ 단일 360° 카메라 + 360° LiDAR로 전 방향 감지 <br>
&nbsp;&nbsp;&nbsp;◦ 추가적인 센서 마운트 고민 없이 플랫폼 독립적 구성

- **ERP 기반 고효율 포인트 매칭**<br>
&nbsp;&nbsp;&nbsp;◦ 3D LiDAR 포인트를 ERP로 투영<br>
&nbsp;&nbsp;&nbsp;◦ 계산량 감소 : 복잡한 3D-3D 매칭 대신 2D bbox 조건 기반 매칭<br>

- **Robust Object Detection**<br>
&nbsp;&nbsp;&nbsp; ◦ DINO: 왜곡이 있는 ERP 이미지에서도 강건한 특징 추출<br>
&nbsp;&nbsp;&nbsp; ◦ OC-SORT: 빠르고 안정적인 multi-object tracking 제공<br>

- **Real-World Industrial Evaluation**<br>
&nbsp;&nbsp;&nbsp;◦ 실제 CJ 산업 공장, 학교 실내, 야간 실외 환경에서 테스트<br>
&nbsp;&nbsp;&nbsp;◦ AP, MOTA, IDF1 기반 정량 평가<br>
&nbsp;&nbsp;&nbsp;◦ 위험도 시각화 기반 실사용 가능성 검증

## 🤖 Used Sensor

<img width="477" height="634" alt="image" src="https://github.com/user-attachments/assets/d494772a-46fe-4727-bab7-867aad4ee700" />
<br>

**1. Ricoh Theta Z1 (360° Camera)**
<br>
**2. Livox Mid-360 (360° LiDAR)**
<br>
**3. Custom Integrated Sensor Mount**
## 🛠 Pineline
<img width="1694" height="567" alt="image" src="https://github.com/user-attachments/assets/305b65ba-ad3f-4851-ba96-387f88517d26" />


### 1. 2D Detection (DINO)
- ERP로 변환된 2D 이미지를 입력 받아 사람 bbox 검출
- Self-attention을 활용해 왜곡에 강한 전역 객체 특징 추출
  
### 2. Tracking (OC-SORT)
- DINO에서 검출된 bbox를 입력받아 프레임 간 ID 유지
- 공장 환경의 많은 가림(occlusion)을 고려하여 강건한 추적
  
### 3. 3D Detection (Ultratics ROS)
- 라이다 포인트를 카메라 프레임으로 변환 (extrinsic 사용)
- ERP(equirectangular)로 3D → 2D 매핑
- 포인트(u, v)가 bbox 내부인지 여부로 매칭
-  매칭된 포인트만 다시 3D 좌표로 복원 → 객체까지 거리 계산
  
## 📊 Results
<img width="1126" height="607" alt="image" src="https://github.com/user-attachments/assets/5f469dd9-f303-4ff2-adc6-c4d90d9024e8" />
<br>

### Detection Performance (AP / CD)
| Environment | AP    | CD(px) |
|:-----------:|:-----:|:------:|
| Factory     | 0.588 | 24     |
| Indoor      | **0.700** | 27 |
| Outdoor(Night)| 0.095 | 30   |

### Tracking Performance (MOTA / IDF1)
| Environment | MOTA    | IDF1 |
|:-----------:|:-----:|:------:|
| Factory       | **89.1** | 83.5    |
| Indoor        | 76.7 | **84.9**    |
| Outdoor(Night)| 53.4 | 67.2  |


## 🔧 Getting Started

### 1. Installation

#### 1. RTX 30 Series
**test : Python=3.8, PyTorch=1.12.1, Torchvision=0.13.1, CUDA=11.6**
<br>
**i. Setting**
```
pip install torch==1.12.1+cu116 torchvision==0.13.1+cu116 \
  --extra-index-url https://download.pytorch.org/whl/cu116
```

**ii. Build package**
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
```
cd ..
mkdir bag
mv ~/your.bag ~/3d_detection/src/ultralytics_ros/bag
```

**iii. Catkin make**
```
cd ~/3d_detection
catkin_make
source devel/setup.bash
```

### 2. How to run
```
roslaunch ultralytics_ros tracking.launch
```
```
roslaunch ultralytics_ros tracker_with_cloud_ros1.launch
```

### 3. Docker
```markdown
# 3D Detection + DINO + OC-SORT (ROS Noetic + Docker)
Ubuntu 20.04 · ROS Noetic · PyTorch 1.12.1 + cu116  
DINO CUDA ops fully prebuilt inside Docker
```
---

### 3.1 Workspace 생성 (Host)
---

```bash
mkdir -p ~/your_ws
cd ~/your_ws
```

---

### 3.2 3d_detection 소스 클론

```bash
cd ~/your_ws
git clone https://github.com/happious/3d_detection.git
```

---

### 3.3 DINO Weights 준비 (Host)

```bash
mkdir -p ~/your_ws/3d_detection/src/ultralytics_ros/DINO/weights
cp ~/Downloads/checkpoint0011_4scale.pth \
   ~/your_ws/3d_detection/src/ultralytics_ros/DINO/weights/
```

---

### 3.4 Bag 파일 준비 (Host)

```bash
mkdir -p ~/your_ws/3d_detection/src/ultralytics_ros/bag
cp ~/CJ.bag \
   ~/your_ws/3d_detection/src/ultralytics_ros/bag/
```

---

### 3.5 Dockerfile 생성

`~/your_ws/Dockerfile` 작성:

```dockerfile
FROM osrf/ros:noetic-desktop-full-focal

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

RUN apt-get update && apt-get install -y \
    git build-essential cmake \
    python3-pip python3-dev \
    python3-rosdep python3-colcon-common-extensions \
    libgl1 libsm6 libxext6 libxrender1 libglib2.0-0 \
    ffmpeg nvidia-cuda-toolkit \
    && rm -rf /var/lib/apt/lists/*

RUN ln -s /usr/bin/python3 /usr/bin/python || true

RUN pip3 install --no-cache-dir --upgrade pip \
 && pip3 install setuptools==59.5.0 wheel \
 && pip3 install importlib-metadata==4.13.0 \
 && pip3 install "typing-extensions<4.6.0"

RUN pip3 install --no-cache-dir \
    torch==1.12.1+cu116 torchvision==0.13.1+cu116 \
    --extra-index-url https://download.pytorch.org/whl/cu116 \
    --no-deps

ENV LD_LIBRARY_PATH=/usr/local/lib/python3.8/dist-packages/torch/lib:${LD_LIBRARY_PATH}

RUN rosdep init || true && rosdep update

ENV CATKIN_WS=/opt/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

COPY 3d_detection/ $CATKIN_WS/src/3d_detection

WORKDIR $CATKIN_WS/src/3d_detection
RUN pip3 install --no-cache-dir --ignore-installed -r requirements.txt \
 && pip3 install "protobuf==3.20.*" \
 && pip3 install "numpy==1.23.5"

WORKDIR $CATKIN_WS
RUN rosdep install --from-paths src --ignore-src -r -y || true

WORKDIR $CATKIN_WS/src/3d_detection/src/ultralytics_ros/DINO/models/dino/ops

RUN sed -i "s/raise NotImplementedError('Cuda is not availabel')/pass/" setup.py

RUN python3 setup.py clean \
 && python3 setup.py build_ext --inplace \
 && python3 -m pip install .

RUN python3 -c "import MultiScaleDeformableAttention as MSDA; print('MSDA import OK:', MSDA)"

ENV PYTHONPATH=/opt/catkin_ws/src/3d_detection/src/ultralytics_ros/DINO/models/dino/ops:${PYTHONPATH}

WORKDIR $CATKIN_WS
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && catkin_make"

RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
 && echo 'source /opt/catkin_ws/devel/setup.bash' >> /root/.bashrc

CMD ["/bin/bash"]
```

---

### 3.6 Docker 이미지 빌드

```bash
cd ~/your_ws
docker build -t 3d_detection_dino .
```

---

### 3.7 컨테이너 실행

```bash
docker run --gpus all -it --name dino_container 3d_detection_dino
```

---

### 3.8 Launch 실행

#### 터미널 1
```bash
docker exec -it dino_container bash
roslaunch ultralytics_ros tracking.launch
```

#### 터미널 2
```bash
docker exec -it dino_container bash
roslaunch ultralytics_ros tracker_with_cloud_ros1.launch
```

---





## 📕 References
[1] DINO : https://github.com/IDEA-Research/DINO
<br>
[2] OC-SORT : https://github.com/noahcao/OC_SORT
<br>
[3] ultralytics_ros : https://github.com/Alpaca-zip/ultralytics_ros
