# UR5e Robotic Chess Player

## Description
The ***UR5e robotic chess player*** is an autonomous system that drives an industrial collaborative 
robotic arm through visual information obtained from a camera to play physical chess against a human.

The challenges for robotic arms operating on a production line include **detection**, **decision**, and **reaction**. 
These challenges are also in making a robotic arm playing chess.
Therefore, developing a system to play chess can give an insight into solving the possible problem in
the application of a robotic arm in a production line

### Hardware
* Universal Robot UR5e collaborative robot
* Robotiq Hand-e gripper
* Allied Vision Mako camera *(mounted on the robotic arm)*
![hardware intro](doc/hardware_intro.png)

### Software
The current software system was developed and tested in Ubuntu 18.04 with ROS melodic. The software only 
works for the hardware listed above. It can be modified to fit different or more hardware for future 
development. ![software overview](doc/software_overview.png)

* UR5e controller: driver for the UR5e manipulator.
* Gripper controller: driver for the Robotiq HandE controller.
* AVT camera controller: driver for the AVT camera.
* Manipulation node: motion planning for manipulation taks.
* Board state detection: running CNN model for piece classification.
* Task planning: high-level task planning node.
* GUI: graphical user interface node.

## Getting started

### Prerequisites
#### 1. Install ROS driver for Universal Robot.
Follow the [documented instruction](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#building) in the offical Universal Robot ROS driver package.
#### 2. Build Universal_Robots_ROS_Driver
```bash
# Assuming your ROS workspace is called my_catkin_ws, inside the home directory.
$ cd ~/my_catkin_ws/src

# clone the driver
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git

# clone fork of the description. This is currently necessary, until the changes are merged upstream.
$ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git

# install dependencies
$ rosdep install --from-paths src --ignore-src -y

# build the workspace. We need an isolated build because of the non-catkin library package.
$ cd ~/my_catkin_ws
$ catkin_make
```
Make sure that the `ros_controllers` and `moveit????` is installed.
```
$ sudo apt-get install ros-melodic-ros-controllers ros-melodic-moveit
```

#### 3. Install AVT camera driver
The camera driver is hosted in a [separate package](https://github.com/macs-lab/avt_camera#usage).
Please follow instruction there to install the camera Vimba driver (required for AVT camea) and clone
the avt_camera package.

If a camera other than the [Mako camera](https://www.alliedvision.com/en/products/cameras/detail/Mako%20G/G-131.html) is used, please create a similar ROS driver that listen to the `/trigger` topic and publish the extracted image data to `/avt_camera_img` topic.

#### 4. Install Hand-E gripper driver
The ROS driver for hand-e gripper is hosted [here](https://github.com/macs-lab/robotiq_hande_ros_driver). Just clone and build the sorce code.
```bash
$ cd ~/my_catkin_ws/src
$ git clone https://github.com/macs-lab/robotiq_hande_ros_driver.git
$ cd ..
$ catkin_make
```
#### 5. Install Python3
ROS melodic was build with python2. In order to use `pytorch` and `pyqt5`, we need to install python3 as well.
```bash
# check which python3 version in the system. If you have already install python3 version >= 3.6, you are good to go.
$ python3 --version
# If the above command shows an error, or you have python3 version < 3.6, install/reinstall python3
$ sudo apt-get install software-properties-common
$ sudo add-apt-repository ppa:deadsnakes/ppa
$ sudo apt-get update
$ sudo apt-get install python3.6
```

#### 6. Install required python library and software
```bash
# install python packages with pip
$ pip install pytransform3d numpy 
```

#### 7. Clone and build source code
```bash
# clone source code
$ cd ~/my_catkin_ws/src
$ git clone https://github.com/macs-lab/defect_inspection_arm.git

# build
$ cd ~/my_catkin_ws
$ catkin_make
```

### Setting Up
#### 1. Setting up UR robot
# need to check
### Setting up UR robot
Please follow [this document](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md) to install the urcap needed to the robot and create the external control program.
### Update calibration file
This package use a [calibration file](https://github.com/macs-lab/defect_inspection_arm/blob/master/external_control_ur5e/etc/ur_calibration_correction.yaml) extracted from the the demo robot. It needs to be updated manually.
```bash
$ roslaunch ur_calibration calibration_correction.launch \
  robot_ip:=<robot_ip> target_filename:="~/my_catkin_ws/src/defect_inspection_arm/external_control_ur5e/etc/ur_calibration_correction.yaml"
```
Update the `robot_ip` parameter to your UR5e's IP address inside [`robot_bringup.launch`](https://github.com/macs-lab/defect_inspection_arm/blob/master/external_control_ur5e/launch/robot_bringup.launch) file.

Now, the UR robot should be able to be controlled with ROS node. To test if the driver is working properly, do the following:
* power up the robot and open the external control program.
* ROS launch `robot_bringup.launch`:
```bash
$ roslaunch external_control_ur5e robot_bringup.launch
```
* Run the external control program on the robot
* Use `rqt_trajectory_controller` to test if the driver is working properly.
```bash
$ rosrun rqt_trajectory_controller rqt_trajectory_controller
```

## 2. Setting up camera driver
Update parameters in the camera launch file (inspection_cell_motion_planning/launch/start_perception_camera_freerun.launch, inspection_cell_motion_planning/launch/start_perception_camera.launch). Specificyly, update the `cam_IP` parameter to the actual camera IP adress and adjust `exposure_in_us` (in microsecond) such that the image is properly exposed.

To test if the camera driver is working properly, do the following:
* launch camera in free run mode.
```bash
$ roslaunch inspection_cell_motion_planning start_perception_camera_freerun.launch
```
* use [`rqt_image_view`](http://wiki.ros.org/rqt_image_view) to see image output.
```bash
$ rosrun rqt_image_view rqt_image_view
```

## 4. Setting up gripper controller
Go to `gripper_bringup.launch`, modify the parameter `robot_ip` to the actual IP address for the UR robot. 

To test if the gripper controller is working properly, launch `gripper_bringup.launch` and run `test.py` inside the `robotiq_hande_ros_driver` pakcage.
```bash
# launch gripper driver
$ roslaunch robotiq_hande_ros_driver gripper_bringup.launch
# open a new bash, run
$ rosrun robotiq_hande_ros_driver test.py
# you should see the gripper moving.
```

## 8. Setting up motion planning node

### 8.1 Camera calibration and hand-eye calibration
Use [`camera_hand_eye_calibration`](https://github.com/xiaohuits/camera_hand_eye_calibration) to obtain the `camera_hand_eye_calibration.yaml` file. 
It contains the camera calibration results as well as the hand-eye position.
Copy the file and put it into `/inspection_cell_motion_planning/yaml` folder.

# Usage
## Bring up UR driver
First, power on the robot. Open a terminal, run the following:
```bash
$ roslaunch external_control_ur5e robot_bringup.launch
```
Then run the external control program (external_control.urp) on the robot.
Back to the terminal, you should see message similar to `robot is ready to receive control command`.

## Bring up grasping node
Open a terminal, run the following:
```bash
$ roslaunch inspection_cell_motion_planning bring_up.launch
```

## Bring up neural network model:
Open a terminal, run:
```bash
$ rosrun defect_classifier defect_classifier_node.py
```

## Run task planning node:
Open a terminal, run:
```bash
$ rosrun inspection_cell_task_planning inspection_task_planning_node.py
```

## Run GUI node:
Open a terminal, run:
```bash
rosrun inspection_cell_task_planning gui_node.py
```
You should see a GUI window similar to the picture below:

![GUI](doc/GUI.png)

* Click `check occupancy` to update the occupancy map for the primary bin.
* Click `start inspection` to start the inspection process.
* Click `stop inspection` to stop the inspection process after inspecting the current part.
* Click `clear secondary bin` to reset the occupancy map for the secondary bin.
