ARG CUDA_IMAGE_TAG
FROM nvidia/cuda:$CUDA_IMAGE_TAG
ARG ROS_DISTRO
ARG CUDNN_VERSION
ARG TENSORRT_VERSION


# Update environment
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        apt-utils \
        gettext-base \
        locales \
        keyboard-configuration && \
    rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8; dpkg-reconfigure -f noninteractive locales
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8


# Install apt packages
COPY apt-packages /tmp/
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        $(cat /tmp/apt-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages


# Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-${ROS_DISTRO}-desktop-full \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-rosdep \
    python3-vcstool \
    && apt-get clean
    

# Init rosdep
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash; \
rosdep init'


# Install pip packages
COPY pip3-packages /tmp/
RUN pip3 install -U \
        $(cut -d# -f1 </tmp/pip3-packages) \
    && rm -rf /root/.cache /tmp/pip-* /tmp/pip3-packages


# Install cuDNN and TensorRT
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
  libcudnn8="$CUDNN_VERSION" \
  libcudnn8-dev="$CUDNN_VERSION" \
  libnvinfer8="$TENSORRT_VERSION" \
  libnvinfer-dev="$TENSORRT_VERSION" \
  libnvinfer-plugin8="$TENSORRT_VERSION" \
  libnvinfer-plugin-dev="$TENSORRT_VERSION" \
  libnvonnxparsers8="$TENSORRT_VERSION" \
  libnvonnxparsers-dev="$TENSORRT_VERSION" \
  libnvparsers8="$TENSORRT_VERSION" \
  libnvparsers-dev="$TENSORRT_VERSION" \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*


# Install a web browser
RUN if [ "$(uname -m)" = "x86_64" ]; then \
        curl -LO https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb && \
        apt-get install -y ./google-chrome-stable_current_amd64.deb && \
        rm google-chrome-stable_current_amd64.deb; \
    fi
 

# Clean up unnecessary files
RUN rm -rf \
  "$HOME"/.cache \
  /etc/apt/sources.list.d/cuda.list \
  /etc/apt/sources.list.d/docker.list \
  /etc/apt/sources.list.d/nvidia-docker.list

    
# Add GitHub to known hosts for private repositories
RUN mkdir -p ~/.ssh \
  && ssh-keyscan github.com >> ~/.ssh/known_hosts


# Customize bash
COPY bashrc-git-prompt /
RUN cat /bashrc-git-prompt >> /etc/skel/.bashrc && \
    rm /bashrc-git-prompt
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/skel/.bashrc


# Install ignition dependencies
RUN git clone https://github.com/ignitionrobotics/ign-math --branch ign-math3 /tmp/ign-math
RUN mkdir /tmp/ign-math/build && \
    cd /tmp/ign-math/build && \
    cmake .. && \
    make -j4 && \
    make install
    
RUN git clone https://github.com/ignitionrobotics/ign-msgs --branch ign-msgs0 /tmp/ign-msgs
RUN mkdir /tmp/ign-msgs/build && \
    cd /tmp/ign-msgs/build && \
    cmake .. && \
    make -j4 && \
    make install


# Copy entrypoint
COPY entrypoint /entrypoint


# Clean up
RUN apt-get update && \
    apt-get clean && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    rm -rf /var/lib/apt/lists/*


ENTRYPOINT ["/entrypoint"]
CMD ["/bin/bash"]
LABEL maintainer="Amadeusz Szymko (amadeuszszymko@gmail.com)"
