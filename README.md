# UR5e Robotic Chess Player

# Description
The ***UR5e robotic chess player*** is an autonomous system that drives an industrial collaborative 
robotic arm through visual information obtained from a camera to play physical chess against a human.

The challenges for robotic arms operating on a production line include **detection**, **decision**, and **reaction**. 
These challenges are also in making a robotic arm playing chess.
Therefore, developing a system to play chess can give an insight into solving the possible problem in
the application of a robotic arm in a production line

## Hardware
* Universal Robot UR5e collaborative robot
* Robotiq Hand-e gripper
* Allied Vision Mako camera *(mounted on the robotic arm)*
![hardware intro](doc/hardware_intro.png)

## Software
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

# Getting started

## Installation
### 1. Install ROS and create a workspace.
Follow the [ROS melodic installation](http://wiki.ros.org/melodic/Installation/Ubuntu) to download the ROS.\
Follow the [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) to create a ROS workspace using `catkin`

### 2. Install AVT camera driver
Go to the [official website](https://www.alliedvision.com/en/products/vimba-sdk/#c1497). Download `Vimba_v5.0_Linux` driver and extract it in `Home` directory. 
Look inside the folder named VimbaGigETL, run `Install.sh` file from command line. After installation, log off once to activate the driver.

Please use `VimbaViewer` to test if your camera has been properly configered and is discoverable. The `VimbaViewer` can be found inside the extracted dowload folder's `Tools/Viewer/Bin/x86_64bit` folder. If you cannot open the camera and grab images using `VimbaViewer`, this ROS wrapper will fail, too.

More information is in this [seperate package](https://github.com/macs-lab/avt_camera#usage)

### 3. Install Python3
Python3 is requred to install for using `pytorch` and `pyqt5`.
```bash
# check which python3 version in the system. If you have already install python3 version >= 3.6, you are good to go.
$ python3 --version
# If the above command shows an error, or you have python3 version < 3.6, install/reinstall python3
$ sudo apt-get install software-properties-common
$ sudo apt-get update
$ sudo apt-get install python3.6
```

### 4. Install used python library and software
* Python library
```bash
$ python -m pip install --user numpy scipy pytransform3d opencv-python==4.2.0.32
$ python3 -m pip install PyQt5 opencv-python-headless rospkg numpy chess
$ sudo apt-get install ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-ros-controllers 
  ros-melodic-moveit
```
* PyTorch\
Follow the PyTorch [GET STARTED](https://pytorch.org/get-started/locally/) to download the pyrotch

### 5. Install cv_bridge for python3
* Download dependency
```bash
$ sudo apt-get install python3-pip python-catkin-tools python3-dev python3-numpy
$ sudo pip3 install rospkg catkin_pkg
```
* Create workspace and clond package
```bash
$ mkdir -p ~/cvbridge_build_ws/src
$ cd ~/cvbridge_build_ws/src
$ git clone -b melodic https://github.com/ros-perception/vision_opencv.git
```
* Compilation
```bash
$ cd ~/cvbridge_build_ws
$ cd catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m 
  -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
$ catkin config --install
$ catkin build cv_bridge
```
* Source package
```bash
$ source install/setup.bash --extend
$ echo "source install/setup.bash --extend" >> ~/.bashrc
```

## Git clone packages
```bash
$ cd ~/catkin_ws/src

# clone `avt_camera` package
$ git clone https://github.com/macs-lab/avt_camera.git

# clone `robotiq_hande_ros_driver` package
$ git clone https://github.com/macs-lab/robotiq_hande_ros_driver.git

# clone `robotic_chess_player` package
$ git clone https://github.com/macs-lab/robotic_chess_player.git

# build
$ cd ~/catkin_ws
$ catkin_make
```

## Setting Up

### 1. Setting up UR robot
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

### 2. Setting up camera driver
Make sure the `cam_IP` parameter in the camera launch file (robotic_chess_player/launch/camera_bringup_freerun.launch) match with the actual camera IP adress. 

To test if the camera driver is working properly, do the following:
* launch camera in free run mode.
```bash
$ roslaunch robotic_chess_player camera_bringup_freerun.launch
```
* use [`rqt_image_view`](http://wiki.ros.org/rqt_image_view) to see image output.
```bash
$ rosrun rqt_image_view rqt_image_view
```

### 3. Setting up gripper controller
Go to `gripper_bringup.launch`, modify the parameter `robot_ip` to the actual IP address for the UR robot. 

To test if the gripper controller is working properly, launch `gripper_bringup.launch` and run `test.py` inside the `robotiq_hande_ros_driver` pakcage.
```bash
# launch gripper driver
$ roslaunch robotiq_hande_ros_driver gripper_bringup.launch
# open a new bash, run
$ rosrun robotiq_hande_ros_driver test.py
# you should see the gripper moving.
```

### 4. Camera calibration and hand-eye calibration
**This step directly affects the pose estimation accuracy!\
Do the calibration when switching the camera or changing the camera's focal length.**\
Use [camera_hand_eye_calibration](https://github.com/xiaohuits/camera_hand_eye_calibration) to obtain the `camera_hand_eye_calibration.yaml` file. 
It contains the camera calibration results as well as the hand-eye position.
Copy the file and put it into `/robotic_chess_player/config` folder.

### 5. Chessboard's square length and compensation of z direction's error 

# Usage
## Start the entire system 
### 1. Bring up UR5e robot driver
(**This is for starting the robotic arm in B012 lab**)
* Power on the robot and load the installation on the robot's polyscope.
* Open a terminal and initiate the driver by following the manual: [Running Universal_Robots_ROS_Driver in a separate machine](https://github.com/macs-lab/lab_doc/wiki/Running-Universal_Robots_ROS_Driver-in-a-separate-machine).
* Run the external control program (external_control.urp) on the robot. Back to the terminal, you should see message similar to `robot is ready to receive control command`.

### 2. Bring up neural network detection system
Open a new terminal, run:
```bash
$ source ~/cvbridge_build_ws/install/setup.bash --extend
$ rosrun robotic_chess_player chessboard_state_detection.py 
```

### 3. Bring up the entire system:
Open a new terminal, run:
```bash
$ roslaunch robotic_chess_player entire_system_bringup.launch 
```
### 4. System instruction
A GUI windonw will show up after finish step 3 and it looks like the image below.

![GUI](doc/GUI.png)

1. Before placing the chess pieces on the board, first, click `Locate Chessboard` to estimate the chessboard pose relative to the robotic arm's base.
2. Place the chess pieces on the board, and the human player makes a move. Then, click `Detect Chessboard` to detect the chessboard state. The result will be shown in the left dialog box.
3. Enter false detections' square and correct piece type in the dialog box on the right and click `Correct Chessboard`. The formate of the comment is first to specify which square in lowercase, press space, and enter the correct piece's type. Each type of chess piece is using one letter. Capital letter means white and lowercase letter represents black. Piece type: k(king), q(queen), r(rook), n(knight), b(bishop), p(pawn).
4. Click `Confirm` to make the system search for the best chess move and perform that move through driving the robotic arm.

