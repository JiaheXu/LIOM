FROM ubuntu:20.04

RUN apt-get update && \
    apt-get install -y \
    curl \
    git \
    vim \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive

RUN ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime
RUN apt-get update && apt-get install -y curl gnupg lsb-release wget

# Install the ROS GPG key for downloads
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Append the ROS sources into the apt lists
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list

# Add the Intel librealsense source and key.
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN echo "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" > /etc/apt/sources.list.d/nea-intel.list;

RUN apt-get clean &&\
    apt-get autoclean &&\
    rm -rf /var/lib/apt/lists/ &&\
    apt update


RUN apt-get update && apt-get install -y ros-noetic-desktop-full
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc

RUN apt-get update \
    && apt install -y --no-install-recommends \
    cmake \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libparmetis-dev \
    && apt-get clean

# COPY copy_files /opt/


# RUN cd /opt/ceres-solver \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && make -j16 \
#     && make install


RUN apt-get update \
    && apt install -y --no-install-recommends \
    cmake \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libparmetis-dev \
    && apt-get clean


RUN cd /tmp \ 
    && wget -q http://ceres-solver.org/ceres-solver-2.0.0.tar.gz \
    && tar zxf ceres-solver-2.0.0.tar.gz \
    && mkdir ceres-bin \
    && cd ceres-bin \
    && cmake ../ceres-solver-2.0.0 \
    && make -j8 \
    && make install

RUN apt-get update && apt-get install -y git

RUN echo "source /usr/share/bash-completion/completions/git" >> /etc/bash.bashrc


RUN cd /tmp \
 && git clone http://github.com/strasdat/Sophus.git \
 && cd Sophus && git checkout 97e7161 \
 && mkdir build && cd build && cmake .. -DBUILD_TESTS=OFF \
 && make -j8 && sudo make install


#Install gtsam - library for localization and mapping
RUN cd /tmp \
    # && wget -O /tmp/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip \
    && wget -O /tmp/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.3.zip \
    && unzip gtsam.zip -d /tmp/ \
    # && cd /tmp/gtsam-4.0.2/ \
    && cd /tmp/gtsam-4.0.3/ \
    && mkdir build \
    && cd build \
    && cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF .. \
    && make install -j8

# RUN cd /opt/fmt \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && make -j16 \
#     && make install

# RUN cd /opt/gtsam-4.0.3 \
#     && mkdir build \
#     && cd build \
#     && cmake .. -DBUILD_TESTS=OFF \
#     && make -j16 \
#     && make install

# RUN cd /opt/opencv-4.5.1 \
#     && mkdir build \
#     && cmake -D CMAKE_INSTALL_PREFIX=/usr/local -D CMAKE_BUILD_TYPE=Release -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_ENABLE_NONFREE=True ..  \
#     && make -j12 \
#     && make install


RUN sudo apt install -y ros-noetic-velodyne-pointcloud

























