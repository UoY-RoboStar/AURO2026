cmake_minimum_required(VERSION 3.17)  # 3.17: AUTOMOC also processes .hh files
project(auro_gz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_gui_vendor REQUIRED)
find_package(gz-gui REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(Qt6 REQUIRED COMPONENTS Core Qml Quick)

# Each plugin is its own library, which SDF refers to by name: filename="gz_link_attacher" loads
# libgz_link_attacher.so

# Server systems
foreach(plugin gz_link_attacher:LinkAttacher gz_model_carrier:ModelCarrier gz_joint_brake:JointBrake)
  string(REPLACE ":" ";" plugin "${plugin}")
  list(GET plugin 0 target)
  list(GET plugin 1 source)
  add_library(${target} SHARED src/${source}.cc)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_link_libraries(${target} gz-sim::gz-sim gz-plugin::register)
  install(TARGETS ${target} LIBRARY DESTINATION lib)
endforeach()

# GUI plugin: the winch cable (filename="WinchCable")
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
add_library(WinchCable SHARED
  src/winch_cable/WinchCable.cc src/winch_cable/WinchCable.hh src/winch_cable/WinchCable.qrc)
target_compile_features(WinchCable PRIVATE cxx_std_17)
target_link_libraries(WinchCable
  gz-gui::gz-gui
  gz-rendering::gz-rendering
  gz-plugin::register
  Qt6::Core
  Qt6::Qml
  Qt6::Quick
)
install(TARGETS WinchCable LIBRARY DESTINATION lib)

# Sourcing install/setup.bash adds <prefix>/lib to GZ_SIM_SYSTEM_PLUGIN_PATH and
# GZ_GUI_PLUGIN_PATH, so the Gazebo server and GUI can find the plugins.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
