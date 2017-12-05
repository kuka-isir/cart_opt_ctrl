cmake_minimum_required(VERSION 2.8.3)
project(cart_opt_ctrl)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    rtt_ros
    rtt_ros_kdl_tools
    trac_ik_lib
    kdl_conversions
    eigen_conversions
    cmake_modules
    qpOASES
    controller_interface
    actionlib
    roscpp
    rospy
    realtime_tools
    control_msgs
    trajectory_msgs
    controller_manager
    message_generation
    geometry_msgs
    krl_msgs
    rtt_roscomm
    dynamic_reconfigure
)


find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_python_setup()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)

add_service_files(
  FILES
  UpdateWaypoints.srv
  GetCurrentPose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

# Orocos typekits
ros_generate_rtt_typekit(cart_opt_ctrl)
ros_generate_rtt_service_proxies(cart_opt_ctrl)

## ROS control and trajectory nodes
add_executable(kdl_trajectory_sender src/kdl_trajectory_sender.cpp)
target_link_libraries(kdl_trajectory_sender ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_library(cart_opt_controllers src/cart_opt_controller.cpp)
target_link_libraries(cart_opt_controllers ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(send_simple_traj src/send_simple_traj.cpp)
target_link_libraries(send_simple_traj ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_library(kpa101 src/kpa101.cpp)

orocos_generate_package(
)

## Orocos control and trajectory components
orocos_component(${PROJECT_NAME} src/cart_opt_comp.cpp src/compute_traj_comp.cpp src/impulse_cart_comp.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${kdl_conversions_LIBRARIES}
   ${USE_OROCOS_LIBRARIES}
#    kpa101
)

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS my_msgs
  DEPENDS_TARGETS rtt_roscomm
)
