# ROS2-jazzy-workshop-01
Robot modeling is the cornerstone of modern robotics. The Unified Robot Description Format (URDF) is an XML-based language used in ROS to describe all aspects of a robot, including its physical structure, joints, and sensors. While URDF is powerful, its true potential is unleashed when combined with Xacro (XML Macros), which allows for the creation of reusable code snippets (macros) and mathematical calculations. This modular approach drastically simplifies the creation of complex robot models and makes it easier to manage and modify them. A solid understanding of these description files is crucial for any robotics developer, as many simulation failures in Gazebo, like unexpected collisions or unstable dynamics, often stem from incorrect robot descriptions, such as improperly defined masses or inertias.

During the modeling process, RViz, the ROS visualization tool, is an invaluable asset. It allows you to visualize your robot's URDF model, check for correct joint properties, and verify that all links, masses, and inertias are properly configured before you even attempt to simulate it. This early-stage validation saves significant time and effort. Ultimately, a properly described robot model is a prerequisite for effective simulation in Gazebo. Gazebo is a powerful 3D robotics simulator that provides a realistic environment to test and validate robot behaviors, from simple movements to complex sensor interactions, and is essential for developing and testing algorithms without the need for physical hardware.

Objectives Upon completion of this lab sheet, you will be able to:

- Create a ROS 2 package and navigate its structure.
- Understand and effectively use URDF, Xacro, and Macro for robot modeling.
- Understand the properties of different joint types and links.
- Familiarize yourself with the RViz visualization tool for model validation.
- Familiarize yourself with the Gazebo simulator and the gz_ros_bridge for communication with ROS 2.

Task 01: Setting up Your ROS 2 Workspace
========================================
This task will guide you through the process of setting up a new ROS 2 workspace and a dedicated package for your robot's description files.

Create Your Workspace Directory
-------------------------------
First, you'll create a new workspace directory for this workshop. Please replace **XX** with your batch number to personalize your workspace name. For example, if you are from the **22nd batch**, your workspace name will be **workshop_22_ws**.

    mkdir -p workshop_xx_ws/src
    
Next, navigate into your newly created workspace directory.

    cd workshop_xx_ws/src

Create the Robot Description Package
------------------------------------
Next, create a new ROS 2 package specifically for your robot's description files. We'll name this package robot_description. The --build-type ament_cmake flag specifies the build system we'll be using.

    ros2 pkg create --build-type ament_cmake robot_description
Add Your Workspace to VS Code
-----------------------------
For easier editing and project navigation, it's recommended to add your workspace's src folder to your VS Code environment.
- Open VS Code.
- Go to File > Add Folder to Workspace...
- select your workshop_XX_ws/src directory.

Create Necessary Folders
------------------------
To maintain a clean and organized package structure, please create three new folders inside your workshop_XX_robot_description package directory: **launch**, **urdf**, and **config**. These folders will be used to store your launch files, URDF/Xacro files, and configuration files, respectively.
.. figure:: images/src_overview.png
   :alt: images/src_overview.png
   :width: 600px
   :align: center

