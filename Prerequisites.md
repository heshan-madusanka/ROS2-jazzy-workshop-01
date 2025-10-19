# Prerequisites and Installation Guide for ROS 2 Workshop
Before participating in this workshop you should understand the core concepts of ROS 2 and have a working development environment. This workshop builds on foundational knowledge and practical skills; without them you will not be able to complete the exercises or follow the demonstrations. You are expected to review and complete the [official Beginner: CLI tools](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html) and [Beginner: Client libraries](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html) tutorials in the ROS 2 documentation prior to this session.
    
Install colcon: colcon is an iteration on the ROS build tools catkin_make, catkin_make_isolated, catkin_tools and ament_tools.

    sudo apt install python3-colcon-common-extensions

XACRO Support: For using the XACRO macro language to build clean, reusable URDF files.

    sudo apt install ros-${ROS_DISTRO}-xacro
Joint State Publisher GUI: This provides a simple GUI with sliders to control the joints of a robot model in RViz. This is essential for testing URDF models.

    sudo apt install ros-${ROS_DISTRO}-joint-state-publisher-gui

Robot State Publisher: This package reads the joint states and publishes the necessary TF2 transforms for the robot model.

    sudo apt install ros-${ROS_DISTRO}-robot-state-publisher

Install Gazebo Harmonic & ROS 2 Integration

[https://gazebosim.org/docs/harmonic/install_ubuntu/](https://gazebosim.org/docs/harmonic/install_ubuntu/)

    sudo apt-get install ros-${ROS_DISTRO}-ros-gz
-----------

    sudo apt install ros-${ROS_DISTRO}-gz-ros2-control
