FROM ros:melodic

ENV CATKIN_WS=/root/catkin_ws
ENV N_PROC = 2

RUN apt-get update && apt-get install -y \
    cmake \
    libatlas-base-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    python-catkin-tools \    
    ros-melodic-image-geometry \
    ros-melodic-pcl-ros \
    ros-melodic-image-proc \
    ros-melodic-tf-conversions \
    ros-melodic-sophus \
    ros-melodic-cv-bridge \
    ros-melodic-tf2-geometry-msgs && \
    # System dependencies
    apt-get install -y \
    build-essential unzip pkg-config autoconf \
    libboost-all-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libvtk6-dev libgtk-3-dev \
    libatlas-base-dev gfortran \
    libparmetis-dev \
    libgl1-mesa-dev libglew-dev \
    python-wstool python-catkin-tools && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y 

RUN cd /root/ && \
    git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    git fetch && \
    git checkout tags/v0.5 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF .. && \
    make -j2 && \
    rm -rf Pangolin


# Setup catkin workspace
RUN mkdir -p $CATKIN_WS/src/vslam/ && \
    cd ${CATKIN_WS} && \
    catkin init && \
    catkin config \
    --extend /opt/ros/melodic \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release && \
    catkin config --merge-devel

# COPY ./ ${CATKIN_WS}/src/vslam

RUN cd $CATKIN_WS/src && \ 
    git clone https://github.com/hellovuong/vslam.git

RUN rosdep update

# RUN source /opt/ros/melodic/setup.bash && \
RUN cd $CATKIN_WS/src && \
    wstool init && \
    wstool merge vslam/vslam.rosinstall   && \
    wstool update
RUN cd $CATKIN_WS && \
    catkin build -j2 catkin_simple && \ 
    catkin build -j2 dbow2_catkin && \ 
    catkin build -j2 g2o_catkin 

RUN cd $CATKIN_WS/src/vslam/ORB_SLAM3  && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF .. && \
    make -j2 && \
    cd ../Vocabulary && \
    tar -xf ORBvoc.txt.tar.gz && \
    rm -rf ORBvoc.txt.tar.gz

WORKDIR ${CATKIN_WS}
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8   
RUN cd ${CATKIN_WS} && \ 
    catkin build -j2 && \
    sed -i '/exec "$@"/i \
    source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
