# ROS2 Publisher and Subscriber

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

  

ROS2 beginner tutorial practice assignment for ENPM808X course at University of Maryland.

  

## Task

- Modify the publisher node to publish a custom string message

- Modify the tutorial code to follow Google C++ Style Guide (with course modifications)

- Run cpplint on your ROS package files and save the output as a text file to the results folder

- Run cppcheck on your ROS package files and save the output as a text file to the results folder

  

## Dependencies

- rclcpp

- stdmsgs

- OS: Ubuntu Linux 22.04

- ROS Version: ROS2 Humble Hawksbill

  

## Build Instructions

  

Navigate to the source folder of the ROS2 workspace

```sh
cd ~/ros2_ws/src
```

Clone the GitHub repository

```sh
git clone https://github.com/akasparm/beginner_tutorials.git
```
Now change the name of the directory to ```cpp_pubsub```
```sh
mv beginner_tutorials cpp_pubsub
```

Now to build the package go to the root of the ROS2 workspace

```sh
cd ~/ros2_ws
```

check the dependencies

```sh
rosdep install -i --from-path src --rosdistro humble -y
```

and build the package
```sh
source /opt/ros/humble/setup.bash
```

```sh
colcon build --packages-select cpp_pubsub
```

  
## Testing the package

```sh
cd ~/ros2_ws
source install/setup.bash
colcon test --event-handlers console_direct+ --packages-select cpp_pubsub
```
  

## For Checking tf

Open 5 terminals and run ```source install/setup.bash``` in all terminals in ```~/ros2_ws/``` directory.

### To start static tf publisher
Terminal 1:
```sh
ros2 run cpp_pubsub talker
```

### To see the transform on terminal
Terminal 2:
```sh
ros2 run tf2_ros tf2_echo world talk
```

### To store the tf tree
Terminal 3:
```sh
ros2 run cpp_pubsub server
```

### To store the tf tree
Terminal 4:
```sh
cd ~/ros2_ws/src/cpp_pubsub/results
ros2 run tf2_tools view_frames
```

### To record a bag file
Terminal 5:
```sh
cd ~/ros2_ws/src/cpp_pubsub/launch
ros2 launch cpp_pubsub rosbag_record_launch.xml bag_record:=1
```
If the argument ```bag_record``` is set to `0` it won't record. To terminate press `Ctrl+C` on the terminal.


### To see info of recorded bag file
```sh
cd ~/ros2_ws/src/cpp_pubsub/results
ros2 bag info all_topics
```


### To play the recorded bag file and see
run ```source install/setup.bash``` in all terminals

Terminal 1:
```sh
cd ~/ros2_ws/
ros2 run cpp_pubsub listener
```

Terminal 2:
```sh
cd ~/ros2_ws/src/cpp_pubsub/results
ros2 bag play all_topics
```


### Launch the nodes
To launch the **publisher** node,

```sh
ros2 run cpp_pubsub talker
```
To launch the **subscriber** node, Open a new terminal and run the below script
```sh
cd ~/ros2_ws
```
```sh
. install/setup.bash
```
```sh
ros2 run cpp_pubsub listener
```
To launch the **server** node, Open a new terminal and run the below script
```sh
cd ~/ros2_ws
```
```sh
. install/setup.bash
```
```sh
ros2 run cpp_pubsub server
```

### Using the launch file

To run the launch file and initate the publisher, subscriber and the service, Open a new terminal and run the below script
```sh
cd ~/ros2_ws
```
```sh
. install/setup.bash
```
```sh
cd ~/ros2_ws/src/cpp_pubsub/launch
```
```sh
ros2 launch launch.yaml frequency:=20.0
```

### Change the ```frequency``` parameter

 Open a new terminal and run the below script
 ```sh
cd ~/ros2_ws
```
```sh
. install/setup.bash
```

```sh
ros2 param set \minimal_publisher freq 5.0
```


## Result Screenshots

Terminal Snapshot
![Terminal](results/Terminal.png)

Logging Informations
![RQT Log](results/rqt_console.png)

RQT node graph
![RQT Graph](results/rqt_node_graph.png)
