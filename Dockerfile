# Use an official ROS 2 base image, for example, osrf/ros:jazzy-desktop
FROM osrf/ros:jazzy-desktop

# Install dependencies for AWS CLI
RUN apt-get update && \
    apt-get install -y \
    curl \
    iputils-ping \
    less \
    python3 \
    python3-pip \
    unzip && \
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

RUN git clone https://github.com/aws-samples/aws-iot-robot-connectivity-samples-ros2 /root/aws-iot-robot-connectivity-samples-ros2

WORKDIR /root/aws-iot-robot-connectivity-samples-ros2/workspace

RUN source /opt/ros/jazzy/setup.bash && colcon build

RUN echo "source /root/aws-iot-robot-connectivity-samples-ros2/workspace/install/setup.bash" >> ~/.bashrc

COPY  ./ros2_ws /root/ros2_ws

CMD ["bash"]