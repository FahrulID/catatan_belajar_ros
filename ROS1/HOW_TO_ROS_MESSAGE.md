Edit this later, too tired

[Source](http://wiki.ros.org/msg)

# To Make

Buat file messages di folder /msg

message_name_1.msg
```
int64 id
float64 yaw
float64 pitch
float64 roll
float64 x
float64 y
float64 z
bool isCentered
bool isPerpendicular
```

message_name_2.msg
```
message_name_1[] msgs
```

CMakeLists.txt
```
find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  message_name_1.msg
  message_name_2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision_d2
  CATKIN_DEPENDS message_generation
#  DEPENDS system_lib
)
``

package.xml
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
<build_depend>message_generation</build_depend>
<build_export_depend>message_generation</build_export_depend>
<exec_depend>message_generation</exec_depend>
```

# To Use

## From Same Package

Python
```
from {nama_package}.msg import message_name_1
from {nama_package}.msg import message_name_2
```

## From Other Package

CMakeLists.txt of the other
```
find_package(catkin REQUIRED COMPONENTS
  {nama_package_msg_from}
  roscpp_serialization
)

add_executable({nama_node} src/{sub_dir}/{node}.cpp)

target_link_libraries({nama_node}
  ${catkin_LIBRARIES}
)

add_dependencies(nama_node ${catkin_EXPORTED_TARGETS})
```

Package.xml of the other
```
<build_depend>{nama_package}</build_depend>
<build_depend>roscpp_serialization</build_depend>
<build_export_depend>{nama_package}</build_export_depend>
<build_export_depend>roscpp_serialization</build_export_depend>
<exec_depend>{nama_package}</exec_depend>
<exec_depend>roscpp_serialization</exec_depend>
```