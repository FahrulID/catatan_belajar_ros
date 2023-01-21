# Running Gazebo

What will you learn here :
* Running Gazebo Simulation

Prerequisites :
* [QGroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)
* [Instalasi Gazebo](INSTALASI_GAZEBO.md)

## Masuk ke folder PX4-Autopilot

```sh
cd /path/to/PX4-Autopilot
make px4_sitl gazebo
```

## Jalankan

```sh
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

Tidak perlu ``` roscore ``` lagi sebab sudah termasuk dalam ``` roslaunch ```

## Jalankan Node Project

Contoh menjalankan node ada di [sini](https://github.com/FahrulID/catatan_belajar_ros/blob/main/HOW_TO_BUILD_ROS_WORKSPACE.md)