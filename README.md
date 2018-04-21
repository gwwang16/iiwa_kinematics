## IIWA Kinematics

Building

Ubuntu 16.04 + ROS Kinetic

Kinematics implementation for the KUKA LBR IIWA R820 (14 Kg).

[//]: # "Image References"

[fk]: ./imgs/forward_kinematics.jpg
[ik]:./imgs/IK_results.jpg

### Getting Started

If you do not have an active ROS workspace, you can create one by:

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Clone this repo into the **src** directory of your workspace:

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/gwwang16/iiwa_kinematics.git
```

Install dependencies

```
$ cd ~/catkin_ws
$ sudo apt-get update
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/iiwa_kinematics/iiwa_arm/scripts
$ chmod +x safe_spawner.sh
$ chmod +x target_spawn.py
$ chmod +x IK_server.py
```

Build the project:

```
$ cd ~/catkin_ws
$ catkin_make
```
Add following to your .bashrc file
```
$ export GAZEBO_MODEL_PATH=~/catkin_ws/src/iiwa_kinematics/iiwa_arm/models
$ source ~/catkin_ws/devel/setup.bash
```


### Forward kinematics demo

For demo mode make sure the demo flag is set to "true" in `inverse_kinematics.launch` file under iiwa_kinematics/iiwa_arm/launch

```
$ roslaunch iiwa_arm forward_kinematics.launch
```
![alt text][fk]


### Launch the project

```
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/iiwa_arm/scripts
$ ./safe_spawner.sh
```

To run your own Inverse Kinematics code change the demo flag described above to "false" and run your code (once the project has successfully loaded) by:

```
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
![alt text][ik]





References:

Udacity kinematics project:

https://github.com/udacity/RoboND-Kinematics-Project

iiwa urdf and gazebo package:

https://github.com/rtkg/lbr_iiwa

iiwa kinematic parameters:

http://www.inacomm2015.ammindia.org/img/94.pdf


