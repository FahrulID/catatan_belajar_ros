# Instalasi

What will you learn here :
* Installing Mavros

[Source](https://docs.px4.io/main/en/ros/mavros_installation.html)

## 1.1 Instalasi Binary

```sh
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
```

## 1.2 Instalasi GeographicLib

```sh
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh  
```

## 1.3 Instalasi Source

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src
```

## 1.4 ( Optional ) ROS Python Tools: wstool, rosinstall, catkin_tools

```sh
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```

### 1.4.1 First time using wstool

```sh
wstool init ~/catkin_ws/src
```

## 1.5 Build

### Install Mavros
#### Mavros Stable

```sh
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
```

#### Mavros Latest

```sh
rosinstall_generator --upstream-development mavros | tee -a /tmp/mavros.rosinstall
```

### Crerate WorkSpace & deps

```sh
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
```

### Install GeographicLib

```sh
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

### Build source

```sh
catkin build
```

### Make sure can run

```sh
#Needed or rosrun can't find nodes from this workspace.
source devel/setup.bash
```