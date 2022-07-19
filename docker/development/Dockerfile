# syntax=docker/dockerfile:1
FROM ubuntu:20.04

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone \
    && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update \
    && apt-get install -q -y --no-install-recommends tzdata \
    && rm -rf /var/lib/apt/lists/*

# Create the image file structure
RUN mkdir /home/catkin_ws \
    && mkdir /home/catkin_ws/src \
    && mkdir /home/rosbags

# Setup the environments
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
ENV HOME=/home/
ENV ROS_WS=/home/catkin_ws/
WORKDIR ${ROS_WS}

# Install some helpful utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
        iputils-ping \
        net-tools \
        netbase \
        python3 \
        python3-pip \
        git \
        nano \
        vim \
        git \
        clang \
        g++ \
        rsync \
        zip \
        make \
        cmake \
        curl \
        wget \
        libopencv-dev \
        python3-opencv \
        build-essential \
        lsb-release \
        ca-certificates \
        dirmngr \
        gnupg2 \
        python3 \
        python3-pip \
        python3-dev \
        netbase \
        htop \
        nmap \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    numpy \
    pandas \
    scipy \
    matplotlib \
    pyserial \
    pymavlink

# Add the ROS repo to the sources list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add the ROS key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS Noetic
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-rosdep \
        python3-rosinstall \
        python3-vcstools \
        ros-noetic-ros-base \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-tf \
        ros-noetic-diagnostic-updater \
        ros-noetic-control-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Install MAVROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-lxml \
        python3-catkin-tools \
        python3-rosinstall-generator \
        geographiclib-tools \
        libgeographic-dev \
	&& geographiclib-get-geoids egm96-5 \
    && geographiclib-get-gravity egm96 \
    && geographiclib-get-magnetic emm2015 \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    pyserial \
    pymavlink

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-control-toolbox

# Initialize the ROS workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ${ROS_WS} \
    && rosdep update \
    && catkin init \
    && wstool init src

# Install MAVROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

# Now install weird MAVROS geoids
RUN sudo /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

# Install REEF Estimator dependencies
RUN apt-get update 
RUN apt-get install google-mock 
RUN apt-get install libgmock-dev 
RUN apt install ros-noetic-eigen-stl-containers 
RUN apt install ros-noetic-tf-conversions -y

#Need this for some reason 
RUN apt-get install ros-noetic-roslint

# Intel realsense camera things
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 
RUN apt-get install software-properties-common -y
RUN apt-get update
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get install librealsense2-dkms -y
RUN apt-get install librealsense2-utils -y
RUN apt-get install ros-$ROS_DISTRO-realsense2-camera -y

#test the camera softare is correct:
#roslaunch realsense2_camera rs_camera.launch

# Copy the reef nodes into the image
#COPY reef_nodes /home/catkin_ws/src/

# Build the ROS workspace
#RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
#    && cd ${ROS_WS} \
#    && catkin_make

# Overlay the ROS environment on top of the existing environment
COPY docker/development/ros_entrypoint.sh /
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]