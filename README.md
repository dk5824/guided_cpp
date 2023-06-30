# ArduPilot_MAVROS
the ROS Package to control UAV with **ArduPilot** in GUIDED Mode using **MAVROS Package**

this package is written with **C++**

System Environment:
Ubuntu 20.04 / ROS Noetic / Gazebo 11 / ArduCopter v 4.3.0

![Result](https://img1.daumcdn.net/thumb/R1280x0/?scode=mtistory2&fname=https%3A%2F%2Fblog.kakaocdn.net%2Fdn%2F92AMQ%2FbtrValB3ztY%2FK5KgEmUWlA3qdtpmlnAZW1%2Fimg.png)


## Install ArduPilot and Gazebo Plugin

### Install ArduPilot Firmware
```bash
git clone https://github.com/ArduPilot/ardupilot
cd ardupilot
git submodule update --init --recursive
```
### Install ArduPilot - Gazebo Plugin
```bash
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

### install extend package
first, install pip tools.

```bash
sudo apt update
sudo apt install python3-pip
```

and using pip tools, install some packages.

```bash
pip3 install pymavlink

sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
pip3 install PyYAML mavproxy --user
echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
```

## Install ROS and MAVROS

### Install ROS Noetic
- ROS Installation: [ROS WIKI ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu)

### Install MAVROS
```bash
sudo apt-get update
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

#install geographicdatasets
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

##  Build Environment Composition

### Install catkin tools

```bash
sudo apt update
sudo apt install python3-osrf-pycommon
sudo apt install python3-catkin-tools
```

### Create Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd .. #move to ~/catkin_ws
catkin_make
source devel/setup.bash

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### Pull This Repository


```bash
cd ~/catkin_ws/src
git clone https://github.com/dk5824/guided_cpp

cd ~/catkin_ws # or "cd .."
catkin_make
source devel/setup.bash
```


## Launch Simulation

### Terminal 1 : Launch Gazebo Simulation
```bash
gazebo --verbose worlds/iris_arducopter_runway.world
```

### Terminal 2 : Run ArduPilot SITL
```bash
cd ardupilot/ArduCopter
python3 ../Tools/autotest/sim_vehicle.py -f gazebo-iris --console
```

### Terminal 3 : Launch MAVROS Node
```bash
roslaunch guided_cpp apm_SITL.launch

# at real UAV
roslaunch guided_cpp apm_Real.launch
```

### Terminal 4 : Launch control node
```bash

roslaunch guided_cpp control.launch
```


## Result

If You Run this package normally, the iris UAV in SITL will takeoff to 5m, and then flying to (2,2,4) point.
Then It will fly to forward direction by 2m/s speed during 10 secs, and then it will be land.

in my ohter repo, **"guided_py"**, you can get more information about mavros package and control functions.

https://github.com/dk5824/guided_py

