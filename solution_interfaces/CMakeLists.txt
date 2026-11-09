cmake_minimum_required(VERSION 3.10)
project(solution_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# The packages of the types the interfaces below use: add a find_package() for each, and list it
# under DEPENDENCIES and as a <depend> in package.xml
find_package(geometry_msgs REQUIRED)

# Every message (msg/*.msg) and service (srv/*.srv) of the solution's own: add new ones here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Example.msg"
  "srv/Example.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
