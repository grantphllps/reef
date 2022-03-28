# syntax=docker/dockerfile:1
FROM arm64v8/ros:noetic-robot

# Ros catkin workspace
RUN mkdir /home/reef_ws \
    && mkdir /home/reef_ws/src

# Setup the environments
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
ENV HOME=/home/
ENV ROS_WS=/home/reef_ws/
WORKDIR ${ROS_WS}

# Install utilities/dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        iputils-ping \
        net-tools \
        netbase \
        python3 \
        python3-pip \
        git \
        nano \
        vim \
        htop \
        python3-lxml \
        python3-catkin-tools \
        python3-rosinstall-generator \
        geographiclib-tools \
        libgeographic-dev \
	&& geographiclib-get-geoids egm96-5 \
    && geographiclib-get-gravity egm96 \
    && geographiclib-get-magnetic emm2015 \
    && rm -rf /var/lib/apt/lists/*

# Install MAVROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

# Now install weird MAVROS geoids
RUN sudo /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

RUN sudo apt-get update

# Install reef things
RUN sudo apt-get install google-mock
RUN sudo apt-get install libgmock-dev
RUN sudo apt install ros-noetic-eigen-stl-containers
RUN sudo apt install ros-noetic-tf-conversions 

# Copy the reef nodes into the image
COPY reef_nodes /home/reef_ws/src/

# Build the ROS workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ${ROS_WS} \
    && catkin_make

# Overlay the ROS environment on top of the existing environment
COPY docker/ros_entrypoint.sh /
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]