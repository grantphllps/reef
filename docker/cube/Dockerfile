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
    scipy \
    matplotlib \
    pyserial \
    pymavlink

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
RUN sudo apt-get update
RUN sudo apt-get install google-mock
RUN sudo apt-get install libgmock-dev
RUN sudo apt install ros-noetic-eigen-stl-containers
RUN sudo apt install ros-noetic-tf-conversions 

#Need this for some reason 
RUN apt-get install ros-noetic-roslint

RUN cd
RUN mkdir /home/librealsense
COPY libraries/librealsense/ /home/librealsense

# RUN cd /home/librealsense
# RUN cp /home/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/ 
# RUN sudo su
# RUN udevadm control --reload-rules && udevadm trigger
# RUN exit
# RUN export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
# RUN source ~/.bashrc

RUN wget https://github.com/PINTO0309/TBBonARMv7/raw/master/libtbb-dev_2018U2_armhf.deb
RUN mv /home/catkin_ws/libtbb-dev_2018U2_armhf.deb /home/
RUN dpkg -i ~/libtbb-dev_2018U2_armhf.deb
RUN ldconfig
RUN rm libtbb-dev_2018U2_armhf.deb
RUN cd ~/librealsense
RUN mkdir  build  && cd build
RUN cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release -DFORCE_LIBUVC=true
RUN make -j1
RUN sudo make install

# Intel realsense camera things
# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 
# RUN apt-get install software-properties-common -y
# RUN apt-get update
# RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
# RUN apt-get install librealsense2-dkms -y
# RUN apt-get install librealsense2-utils -y
# RUN apt-get install ros-$ROS_DISTRO-realsense2-camera -y

# Copy the reef nodes into the image
COPY reef_nodes /home/reef_ws/src/

# Build the ROS workspace
#RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
#    && cd ${ROS_WS} \
#    && catkin_make -j1

# Overlay the ROS environment on top of the existing environment
COPY docker/cube/ros_entrypoint.sh /
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]