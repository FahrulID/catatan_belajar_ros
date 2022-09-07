# Installation

[Source](https://docs.px4.io/main/en/simulation/gazebo.html)

Prerequisites :
* [ROS NOETIC](INSTALASI_ROS_NOETIC.md)

## 1.1 Download PX4 Source Code

```sh
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

## 1.2 Run Ubuntu.sh

```sh
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

# Running

## 1.1 

```sh
    cd /path/to/PX4-Autopilot
    make px4_sitl gazebo
``