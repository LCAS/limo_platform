# syntax = devthefuture/dockerfile-x:v1.4.2
ARG BASE_IMAGE=lcas.lincoln.ac.uk/lcas/ros:jammy-humble-cuda12.2-opengl-1

FROM ${BASE_IMAGE} AS base

# making the standard global variables available for target-specific builds
ARG TARGETARCH

USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN echo "building for ${TARGETARCH}"
RUN if [ "${TARGETARCH}" = "arm64" ]; then \
    add-apt-repository -s -y ppa:openrobotics/gazebo11-non-amd64; \
    fi

COPY .docker/apt-packages.lst /tmp/apt-packages.lst
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -qq -y --no-install-recommends \
        `cat /tmp/apt-packages.lst` && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

FROM base AS vendor_base

INCLUDE .docker/ydlidar.dockerfile
INCLUDE .docker/glog.dockerfile
INCLUDE .docker/magic_enum.dockerfile
INCLUDE .docker/uvc.dockerfile


# get the source tree and analyse it for its package.xml only
FROM base as sourcefilter
RUN mkdir -p /tmp/src/
COPY ./src/*/package.xml /tmp/src/_workspace/src/_pkgs_xmls
COPY ./src/*/*/package.xml /tmp/src/_workspace/src/_pkgs_xmls
COPY ./src/*/*/*/package.xml /tmp/src/_workspace/src/_pkgs_xmls
COPY ./src/*/*/*/*/package.xml /tmp/src/_workspace/src/_pkgs_xmls
COPY ./.docker/*.repos* /tmp/.docker/
RUN cd /tmp/src && for r in /tmp/.docker/*.repos; do vcs import < $r ; done
RUN if [ "${TARGETARCH}" = "arm64" ]; then \
        cd /tmp/src; \
        vcs import < /tmp/.docker/gazebo_ros_pkgs.repos-arm64; \
    fi
RUN cd /tmp/src && vcs pull

# remove everything that isn't package.xml
RUN find /tmp/src -type f \! -name "package.xml" -print | xargs rm -rf

# install all dependencies listed in the package.xml
FROM vendor_base as depinstaller
# copy the reduced source tree (only package.xml) from previous stage
COPY --from=sourcefilter /tmp/src /tmp/src
RUN rosdep update --rosdistro=${ROS_DISTRO} && apt-get update
RUN rosdep install --from-paths /tmp/src --ignore-src -r -y && rm -rf /tmp/src && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/src

FROM depinstaller as depbuilder
COPY .docker/*.repos* .docker/*.sh /tmp/.docker/

# get the source tree and build it (include gazebo only for arm64 platform as it is not available for arm64)
# see https://github.com/gazebosim/gazebo-classic/issues/3236
RUN mkdir -p /opt/ros/lcas/src && \
    cd /opt/ros/lcas/src && \
    for r in /tmp/.docker/*.repos; do vcs import < $r ; done
RUN  if [ "${TARGETARCH}" = "arm64" ]; then cd /opt/ros/lcas/src; vcs import < /tmp/.devcontainer/gazebo_ros_pkgs.repos-arm64; fi

RUN . /opt/ros/humble/setup.sh && \
    apt update && \
    rosdep --rosdistro=${ROS_DISTRO} update && \
    cd /opt/ros/lcas/src && \
    vcs pull && \
    rosdep install --from-paths . -i -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# build the workspace but only until limo_gazebosim to avoid building the hardware specific packages
RUN cd /opt/ros/lcas; colcon build && \
    rm -rf /opt/ros/lcas/src/ /opt/ros/lcas/build/ /opt/ros/lcas/log/

FROM depbuilder as devcontainer

RUN echo "source /opt/ros/lcas/install/setup.bash" >> /etc/bash.bashrc
COPY .docker/bash.alias /tmp/bash.alias
RUN cat /tmp/bash.alias >> /etc/bash.bashrc

FROM devcontainer as compiled

COPY ./src /opt/ros/lcas/src/local-code/src
RUN . /opt/ros/lcas/install/setup.sh && \
    apt update && \
    rosdep --rosdistro=${ROS_DISTRO} update && \
    rosdep install --from-paths /opt/ros/lcas/src/local-code/src --ignore-src -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN cd /opt/ros/lcas && colcon build && \
    rm -rf /opt/ros/lcas/src/ /opt/ros/lcas/build/ /opt/ros/lcas/log/

USER ros
ENV SHELL=/bin/bash
