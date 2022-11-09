# Installation

What will you learn here :
* Installing RealSense SDK (realsense-viewer included)
* Installing RealSense ROS Wrapper For ROS1
* Flattening RealSense Fisheye

[Source](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)
[Source](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

Prerequisites :
* [ROS NOETIC](INSTALASI_ROS_NOETIC.md)

# Realsense SDK

## 1.1 Register Server PublicKey

```sh
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```

## 1.2 Tambahkan Server ke repository

```sh
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
```

## 1.3 Install Libraries

```sh
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
```

## 1.4 Install Developer dan Debug Packages

```sh
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

With dev package installed, you can compile an application with librealsense using ```g++ -std=c++11 filename.cpp -lrealsense2``` or an IDE of your choice.

## 1.5 Jalankan Realsense Viewer

Untuk memverifikasi berhasilnya penginstallan SDK, jalankan RealSense Viewer

* Note : Jangan lupa RealSense Camera dilepas dan pasang kembali

```sh
realsense-viewer
```

# ROS Wrapper for ROS1

Untuk ROS1, gunakan repos [ini](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)

Pastikan sudah menginstall RealSense SDK

## 2.1 Buat Catkin Workspace

* Note : `realsense_ws` dapat diganti dengan apa saja, usahakan bedakan dengan workspace lain, soalnya terkadang ```catkin_make``` konflik dengan ```catkin build```.

```sh
mkdir -p ~/realsense_ws/src
cd ~/realsense_ws/src/
```

## 2.2 Clone Ros Wrapper

```sh
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
```

## 2.3 Build Ros Wrapper

* Note : Bila ada warning, dilewatkan saja, asal bisa ter-built

```sh
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
```

Known error :

* Could not find a package configuration file provided by "ddynamic_reconfigure" with any of the following names:
```sh
sudo apt-get install ros-$ROS_DISTRO-ddynamic-reconfigure
```

## 2.4 Sourcing ke ~/.bashrc

* Note : ingat mengganti `realsense_ws` bila di-custom

```sh
echo "source ~/realsense_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2.5 Starting Camera Node

Cek apakah sudah berhasil menginstall ROS Wrapper Realsense

```sh
roslaunch realsense2_camera rs_camera.launch
```

## 2.6 Menambahkan Topic fisheye

Pertama, ke folder launch

```sh
cd src/realsense-ros/realsense2_camera/launch
```

Edit file Launch yang diinginkan, defaultnya `rs_camera.launch` dengan code editor kesukaanmu

```sh
code rs_camera.launch
# Atau
vim rs_camera.launch
# Atau
nano rs_camera.launch
# Atau
gedit rs_camera.launch
```

Edit line berikut :

```
  <arg name="fisheye_width"       default="-1"/>
  <arg name="fisheye_height"      default="-1"/>
  <arg name="enable_fisheye"      default="false"/>  
```

menjadi

```
  <arg name="fisheye_width"       default="800"/>
  <arg name="fisheye_height"      default="848"/>
  <arg name="enable_fisheye"      default="true"/>  
  <arg name="enable_fisheye1"     default="true"/>
  <arg name="enable_fisheye2"     default="true"/>
```

Kemudian, edit line berikut :  

```
      <arg name="fisheye_width"            value="$(arg fisheye_width)"/>
      <arg name="fisheye_height"           value="$(arg fisheye_height)"/>
      <arg name="enable_fisheye"           value="$(arg enable_fisheye)"/>
```

menjadi

```
      <arg name="fisheye_width"            value="$(arg fisheye_width)"/>
      <arg name="fisheye_height"           value="$(arg fisheye_height)"/>
      <arg name="enable_fisheye"           value="$(arg enable_fisheye)"/>
      <arg name="enable_fisheye1"           value="$(arg enable_fisheye1)"/>
      <arg name="enable_fisheye2"           value="$(arg enable_fisheye2)"/>
```

# Additional

## Flattening

### 3.1 Download flatten_fisheye

Downloadlah `flatten_fisheye`

### 3.2 Jalankan Realsense Node dan Mavros Host

Jalankan di terminal yang berbeda

```sh
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

```sh
roslaunch realsense2_camera rs_camera.launch
```

### 3.3 chmod all files

```sh
chmod +x calculate.py capture.py test.py test_hidden.py
```

### 3.4 Capturing

Jalankan `capture.py`

```sh
python3 capture.py
```

Posisikan Checkerboard di tempat-tempat yang mengalami distorsi ( semakin banyak semakin baik, dan jangan lupa di tempat yang tidak terdistorsi juga ). File checker board ada di [sini](files/). Untuk men-capture tekan tombol `spasi` dan apabila sudah selesai tekan tombol `esc`. 

### 3.5 Kalkulasi

Jalankan `calculate.py`

```sh
python3 calculate.py
```

Akan keluar 3 hasil kalkulasi, yakni DIM, K dan D. Bisa dimasukkan ke dalam file `test.py` maupun `test_hidden.py`

### 3.6 Testing

Jangan lupa mengganti DIM, K dan D.

Jalankan `test.py`

```sh
python3 test.py
```