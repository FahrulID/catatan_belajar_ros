# How To Build

{var} : Change with own var

Prerequisites :
* [MAVROS](INSTALASI_ROS_MAVROS.md)

## Pilih Folder Project

## Buat workspace 

```sh
    mkdir -p catkin_ws/src
    cd catkin_ws/src
```

## Buat Package

```sh
    catkin_create_pkg {nama_package} std_msgs rospy roscpp
```

contoh :

```sh
    catkin_create_pkg {nama_package} geometry_msgs mavros_msgs roscpp rospy std_msgs
```

## Buat Node 

```sh
    cd {nama_package}/src
    touch {nama_node}.cpp || touch {nama_node}.py
```

## Isi Node

Misal dengan [ini](https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html)

Misal pakai di atas, sesuaikan node dengan line >> ros::init(argc, argv, "offb_node");

## Edit CMakeList

### Untuk Python

```sh
    catkin_package(
	    CATKIN_DEPENDS
	    roscpp 
	    rospy
	    std_msgs
	)

    ##

	catkin_install_python(PROGRAMS
	  src/{nama_node}.py
	  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  
	)
```

### Untuk CPP

```sh
    add_executable({nama_node} src/{nama_node}.cpp)
	
    ##

    target_link_libraries(nyoba
	  ${catkin_LIBRARIES}
	)
```

untuk cpp, apabila ada error saat include header, tambahkan dua baris ini pada path c_cpp_properties.json
    "/opt/ros/noetic/include/**",
    "/usr/include/**"

## Ke folder catkin_ws

Run

```sh
    catkin_make || catkin build
```

## Source setup.bash

```sh
    source devel/setup.bash
```

## Jalankan

Jika stuck saat menjalanakan offb_node, fix dengan :
QGroundControl >> Setting >> Parameters >> COM_RCL_EXCEPT >> centang mission dan offboard

contoh

```sh
    rosrun {nama_package} {nama_node}
```