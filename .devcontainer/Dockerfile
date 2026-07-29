ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo bash-completion git vim \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-compressed-image-transport python3-colcon-common-extensions python3-pip python3-opencv feh wget ca-certificates
RUN rm -rf /etc/apt/apt.conf.d/docker-clean

# PlaceNav Python runtime (including the pinned CUDA 11.3 PyTorch wheels).
COPY requirements.txt /tmp/placenav-requirements.txt
RUN printf 'typing-extensions==4.13.2\n' > /tmp/placenav-constraints.txt \
    && python3 -m pip install --no-cache-dir -c /tmp/placenav-constraints.txt -r /tmp/placenav-requirements.txt --extra-index-url https://download.pytorch.org/whl/cu113
ENV SHELL /bin/bash
RUN mkdir -p /home/user/ws/third_party /home/user/ws/src \
    && chown -R $USERNAME:$USERNAME /home/user/ws
COPY src/placenav_viz_msgs/src /tmp/placenav_viz_msgs_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /tmp/placenav_viz_msgs_ws && catkin_make"
RUN cp -a /tmp/placenav_viz_msgs_ws/devel /opt/placenav_viz_msgs_devel
ENV PATH="/home/user/.local/bin:$PATH"
ENV PYTHONPATH="/home/user/ws/src/PlaceNav/src:/opt/placenav_viz_msgs_devel/lib/python3/dist-packages:$PYTHONPATH"

USER $USERNAME
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /home/user/ws/install/setup.bash ]; then source /home/user/ws/install/setup.bash; elif [ -f /home/user/ws/install/local_setup.bash ]; then source /home/user/ws/install/local_setup.bash; fi" >> ~/.bashrc
RUN echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
RUN echo 'export PYTHONPATH="$HOME/ws/src/PlaceNav/src:$PYTHONPATH"' >> ~/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/home/user/ws" >> ~/.bashrc
RUN echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --event-handlers console_direct+'" >> ~/.bashrc

# For ROS 1
RUN echo "export ROSCONSOLE_FORMAT=""'"'[${severity}] [${time}] [${node}]: ${message}'"'" >> ~/.bashrc
RUN echo "export ROSCONSOLE_STDOUT_LINE_BUFFERED=1" >> ~/.bashrc

CMD ["/bin/bash"]
