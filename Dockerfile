FROM ros:kinetic
MAINTAINER Ryota Sakamoto <saka_ro@yahoo.co.jp>

RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get upgrade -y && \
    apt-get install ros-kinetic-cv-bridge git python-pip -y && \
    pip install -U pip

COPY . /opt/ros_ws/src/ros_audit_image
WORKDIR /opt/ros_ws

RUN pip install -r src/ros_audit_image/requirements.txt && \
    source /opt/ros/kinetic/setup.bash && \
    /opt/ros/kinetic/bin/catkin_make && \
    echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros_ws/devel/setup.bash" >> /root/.bashrc
RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh
RUN source /opt/ros_ws/src/ros_audit_image/.env
