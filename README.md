# ROS2 CPP Publisher and Subscriber Tutorial

## Overview

The primary functionality of this package is to publish custom string messages.

## Dependencies

Before running this package on a third-party system, make sure that the following prerequisites are met:

- Ubuntu 22.04
- ROS 2 Humble
- Your ROS 2 workspace is set up correctly

## Build and Run Instructions

1. **Clone Package**

   You can obtain the package by either cloning the repository or copying it to your ROS 2 workspace. Make sure to place it in the `src` directory of your workspace.

   ```sh
   cd ~/ros2_ws/src
   # or Give path to your original ros2 workspace directory

   git clone https://github.com/akasparm/beginner_tutorials.git
   ```

2. **Build the Package**

    Navigate to your ROS 2 workspace and build the package using colcon.

    ```sh
    mv beginner_tutorials cpp_pubsub
    cd cpp_pubsub
    source /opt/ros/humble/setup.bash
    cd ~/ros2_ws
    colcon build --packages-select cpp_pubsub
    ```

3. **Source the Workspace**
    Source the ROS 2 workspace to set the environment for the package.

    ```sh
    . install/setup.bash
    ```

4. **Run the Publisher Node(talker)**
    You can now run the custom string publisher node.

    ```sh
    ros2 run cpp_pubsub talker
    ```

5. **Run the Subscriber Node(listener)**
    Open a new terminal, navigate to ros2 workspace and run the subscriber node.

    ```sh
    cd ~/ros2_ws
    . install/setup.bash
    ros2 run cpp_pubsub listener
    ```

## References

[1] <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html>