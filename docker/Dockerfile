FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y python3-pip \
    git \
    python3-colcon-common-extensions \
    nano \
    xauth \
    libgl1 \
    pkg-config

ENV USERNAME=developer
RUN addgroup realtime
# we use uid of 1000, seems to be default in ubuntu but change if different
RUN useradd -U --uid 1000 -ms /bin/bash $USERNAME \
    && echo "$USERNAME:$USERNAME" | chpasswd \
    && adduser $USERNAME sudo \
    && adduser $USERNAME dialout \
    && adduser $USERNAME realtime \
    && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Commands below run as the developer user
USER $USERNAME

WORKDIR /home/$USERNAME
# building pybullet freezes the raspberrypi so manually limit parallelization
RUN git clone --depth 1 --branch 3.25 https://github.com/bulletphysics/bullet3/
WORKDIR /home/$USERNAME/bullet3
RUN sed -i -e 's%make -j $(command nproc 2>/dev/null || echo 12)%make -j 2%g' build_cmake_pybullet_double.sh
RUN cat build_cmake_pybullet_double.sh
RUN bash build_cmake_pybullet_double.sh
ENV PYTHONPATH=/home/$USERNAME/bullet3/build_cmake/examples/pybullet

WORKDIR /tmp
COPY docker/requirements.txt ./requirements.txt
RUN pip install -r requirements.txt
ENV PATH=$PATH:/home/$USERNAME/.local/bin

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME
RUN mkdir -p /home/$USERNAME/ros_ws/src
ENV MAKEFLAGS="-j4"
SHELL ["/bin/bash", "-c"]
RUN cd /home/$USERNAME/ros_ws/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN sudo apt-get update && sudo apt-get install -y ros-humble-python-orocos-kdl-vendor ros-humble-orocos-kdl-vendor ros-humble-kdl-parser

# installing franka stuff
RUN sudo apt update && sudo apt install -y build-essential cmake git libpoco-dev libeigen3-dev \
    && git clone https://github.com/frankarobotics/libfranka.git \
    && cd libfranka \
    && git checkout 0.9.2 \
    && git submodule init \
    && git submodule update \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j4 \
    && sudo make install
