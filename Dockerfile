FROM ros:jazzy

WORKDIR /workspace

RUN apt-get update && sudo apt-get upgrade -y && apt-get install -y ros-jazzy-rosbridge-suite ros-jazzy-ros2-control libgpiod-dev ros-jazzy-navigation2 ros-jazzy-nav2-bringup

RUN mkdir -p /workspace/src

COPY . /workspace/src

RUN . /opt/ros/jazzy/setup.sh && colcon build
RUN echo ". /opt/ros/jazzy/setup.bash && . /workspace/install/setup.bash" >> ~/.bashrc

# CMD ["bash", "-c", ". /opt/ros/jazzy/setup.bash && . /workspace/install/setup.bash && ros2 launch all robot.launch.py"]


CMD ["bash"]
