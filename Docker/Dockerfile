# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
FROM osrf/ros:noetic-desktop-full-focal

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Install terminator
RUN apt-get update && apt-get install -y --no-install-recommends \
    terminator \
    && rm -rf /var/lib/apt/lists/*

# Install software-properties-common for add-apt-repository
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    && add-apt-repository ppa:git-core/ppa \
    && apt-get update && apt-get install -y --no-install-recommends \
    git \
    && rm -rf /var/lib/apt/lists/*

# Clone the repository and build it
RUN git clone https://github.com/victorkawai/AutonomousDrivingROS.git /root/AutonomousDrivingROS

# Install wget and unzip
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Download and uncompress the ZIP file
RUN wget https://syncandshare.lrz.de/dl/fiLvgiTXetubiN1i4PRjuR/AutonomousDriving.zip -O /root/AutonomousDriving.zip \
    && unzip /root/AutonomousDriving.zip -d /root/ \
    && rm /root/AutonomousDriving.zip

# Remove specific files in the simulation folder and replace with new ones
RUN cp -r /root/Car_build_Data /root/AutonomousDrivingROS/ros_ws/devel/lib/simulation/ \
    && cp /root/Car_build.x86_64 /root/AutonomousDrivingROS/ros_ws/devel/lib/simulation/ \
    && cp /root/UnityPlayer.so /root/AutonomousDrivingROS/ros_ws/devel/lib/simulation/

# Set environment variable to noninteractive to avoid prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary dependencies
RUN apt-get update && apt-get install -y  \
    python3-catkin-tools \
    python3-rosdep \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    xorg \
    xauth \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep only if it has not been initialized
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Build the workspace
RUN cd /root/AutonomousDrivingROS/ros_ws \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build && source devel/setup.bash"

# Set the working directory
WORKDIR /root/AutonomousDrivingROS/ros_ws

# Source the setup script
RUN echo "source /root/AutonomousDrivingROS/ros_ws/devel/setup.bash" >> ~/.bashrc

# Allow access to X server
RUN touch /root/.Xauthority

# Set the default command to run when starting the container
CMD ["/bin/bash"]

