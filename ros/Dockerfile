# Use an official ROS 2 base image, for example, osrf/ros:jazzy-desktop
FROM osrf/ros:jazzy-desktop

# install dev dependencies, not for deployment
RUN apt-get update && \
    apt-get install -y \
    iputils-ping \
    net-tools \
    vim \
    nano && \
    rm -rf /var/lib/apt/lists/*

# Install dependencies for AWS CLI
RUN apt-get update && \
    apt-get install -y \
    curl \
    less \
    python3 \
    python3-pip \
    unzip && \
    rm -rf /var/lib/apt/lists/*

# install dependencies for KVS
RUN apt-get update && \
    apt-get install -y \
    pkg-config \
    m4 \
    libssl-dev \
    libcurl4-openssl-dev \
    liblog4cplus-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base-apps \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    cmake && \
    rm -rf /var/lib/apt/lists/*

RUN pip install awsiotsdk --break-system-packages

ENV PATH="/opt/venv/bin:$PATH"

# Download and install AWS CLI v2
RUN curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip" && \
    unzip awscliv2.zip && \
    sudo ./aws/install && \
    rm -rf awscliv2.zip aws

# Verify AWS CLI installation
RUN aws --version

SHELL [ "/bin/bash", "-c" ]

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

RUN git clone https://github.com/aws-samples/aws-iot-robot-connectivity-samples-ros2 /home/ros2-samples

WORKDIR /home/ros2-samples/workspace

RUN source /opt/ros/jazzy/setup.bash && colcon build

RUN echo "source /home/ros2-samples/workspace/install/setup.bash" >> ~/.bashrc

WORKDIR /home/ros

CMD ["bash"]