# ROS Package

What will you learn here :
* What is Package
* What is Node
* What is Topic
* What is Publisher Node
* What is Subscriber Node
* Topic Example

## Apa itu Package
Package merupakan suatu unit paling atomic untuk ROS. Package itulah yang akan me-wrap beberapa Node sehingga dapat dijalankan. Biasanya Package akan berada di dalam ROS Workspace ( Di dalam Workspace dapat ditambahkan beberapa Package yang berbeda ). 

Salah satu contoh Package adalah MAVROS

### Struktur Package
* include/{package_name} : Merupakan Include Headers untuk C++ ( Pastikan diexport pada CMakeLists.txt )
* msg/ : Folder yang berisikan tipe2 Message ( msg ) [Source](http://wiki.ros.org/msg)
* srv/ : Folder berisikan tipe2 Service ( srv ) [Source](http://wiki.ros.org/srv)
* scripts/ : Berisikan Skrip2 executable
* src/{package_name}/ : File2 Source, terkhususnya Python source yang diekspor ke package lain
* package.xml : Merupakan informasi mengenai Package

Paling sering dipakai :

* src/ : Berisikan Node2 yang akan dibuat
* CMakeLists.txt : Merupakan file CMake build

## Apa itu Node
Node merupakan sebuah program yang dapat dijalankan. Dalam node dapat dilakukan komputasi bagi Robot. Untuk berkomunikasi antar Node dapat menggunakan Publisher dan Subscriber. Dimana Publisher Node akan mengirimkan data sedangkan Subscriber Node akan menerima data. Data yang dimaksudkan merupakan sebuah Topic,.

## Apa itu Topic
Data yang dimaksudkan merupakan sebuah Topic, layaknya di dunia nyata seperti pembicaraan mengenai suatu topik akan ada yang berbicara ( memberi informasi ) dan yang mendengarkan ( mendapat informasi )

Setiap Topic memiliki tipe datanya masing2.

## Apa itu Publisher Node
Suatu node dapat berfungsi sebagai Publisher, dimana Node ini akan mengirimkan data yang diinginkan.

## Apa itu Subscriber Node
Suatu node dapat berfungsi sebagai Subscriber, dimana Node ini akan menerima data yang diinginkan.

## Contoh Topic

Untuk melihat macam2 Topic yang tersedia, jalankan command berikut :

```sh
    rostopic list
```
MAVROS :

Untuk mendapatkan posisi local dari drone, dapat menggunakan :
* mavros/local_position/pose

Topic di atas memiliki tipe data Pose, yang memiliki informasi mengenai posisi dari drone

Untuk menggerakan drone, dapat menggunakan :
* mavros/setpoint_position/local

Topic di atas dapat digunakan untuk memindahkan drone

[Berikut](https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html) adalah contoh menerbangkan drone pada Gazebo dengan menggunakan Publisher Node
[Berikut](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) Contoh membuat Publisher dan Subscriber Node pada Python 2.9
[Berikut](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) Contoh membuat Publisher dan Subscriber Node pada C++