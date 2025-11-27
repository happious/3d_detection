# 3d_detection + DINO (RTX 30 Series)
#  - Ubuntu 20.04 + ROS Noetic
#  - CUDA 11.6 + nvcc
#  - PyTorch 1.12.1 + cu116
FROM nvidia/cuda:11.6.2-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# 1) Base packages + ROS Noetic
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    git \
    build-essential \
    cmake \
    python3-pip python3-dev \
    libgl1 libsm6 libxext6 libxrender1 libglib2.0-0 \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

# 2) Python environment
RUN pip3 install --no-cache-dir --upgrade pip \
 && pip3 install --no-cache-dir \
    setuptools==59.5.0 \
    importlib-metadata==4.13.0 \
    "typing-extensions<4.6.0" \
    wheel

# 3) PyTorch 1.12.1 + cu116
RUN pip3 install --no-cache-dir \
    torch==1.12.1+cu116 torchvision==0.13.1+cu116 \
    --extra-index-url https://download.pytorch.org/whl/cu116 \
    --no-deps

ENV CUDA_HOME=/usr/local/cuda
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# 4) Catkin workspace
ENV CATKIN_WS=/opt/catkin_ws
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}/src

# 5) Copy 3d_detection source
COPY 3d_detection/ ${CATKIN_WS}/src/3d_detection

# 6) Install Python requirements
WORKDIR ${CATKIN_WS}/src/3d_detection
RUN pip3 install --no-cache-dir --ignore-installed --no-deps -r requirements.txt \
 && pip3 install --no-cache-dir "protobuf==3.20.*" "numpy==1.23.5" \
 && pip3 install --no-cache-dir "requests<3" "tomli<2" "platformdirs<4"

# 7) ROS dependencies
WORKDIR ${CATKIN_WS}
RUN rosdep install --from-paths src --ignore-src -r -y || true

# 8) catkin_make
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && catkin_make"


WORKDIR ${CATKIN_WS}
CMD ["/bin/bash"]
