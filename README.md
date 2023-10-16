# ROS2 DDS Raw Proxy

## Why

Sending messages in ROS2 to DDS network need a lot of configurations. So why don't create a DDS node in ROS2 app to forward.

## Get start

### Get ROS2 Raw Proxy

```
git clone https://github.com/wanghaEMQ/ros2-ddsrawproxy.git
```

### Go to your ROS2 work workspace and create a package

```
cd /path/to/ros2_ws/src
ros2 pkg create --build-type ament_cmake nanomq_if
```

### Prepare your .msg

Here we provide a Ddstype.msg as an exmaple

```
cd nanomq_if
mkdir msg
echo "bool      bool_test
int8      int8_test
uint8     uint8_test
int16     int16_test
uint16    uint16_test
int32     int32_test
uint32    uint32_test
int64     int64_test
uint64    uint64_test
float32   float32_test
float64   float64_test

# array example
uint8[256] message" > msg/Ddstype.msg
```

### Edit your CMakeLists.txt and package.xml

CMakeLists.txt

```
cmake_minimum_required(VERSION 3.8)
project(nanomq_if)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ddstype.msg"
)

ament_package()
```

Package.xml

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>nanomq_if</name>
  <version>0.0.0</version>
  <description>Test msg from NanoMQ</description>
  <maintainer email="wanghamax@gmail.com">wangha</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>

<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

</package>
```

### Build your .msg

```
cd /path/to/ros2_ws/src
colcon build --packages-select nanomq_if
source install/setup.bash
ros2 interface show nanomq_if/msg/Ddstype
```

### Find the .idl and copy to ddsrawproxy project

```
cd /path/to/ros2_ws/src
find . -iname "Ddstype.idl"
```

The result might be

```
./install/nanomq_if/share/nanomq_if/msg/Ddstype.idl
./build/nanomq_if/rosidl_adapter/nanomq_if/msg/Ddstype.idl
```

Now. Copy to this project.

```
cp ./install/nanomq_if/share/nanomq_if/msg/Ddstype.idl /path/to/ros2-ddsrawproxy/Ddstype.idl
```

### Build ROS2DDSRawProxy

```
cd /path/to/ros2-ddsrawproxy/../
colcon build --packages-select ros2-ddsrawproxy
```

### Start ROS2DDSRawProxy

```
source ./install/setup.bash
ros2 run rawproxy rawproxy
```


