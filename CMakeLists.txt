cmake_minimum_required(VERSION 2.8.3)
project(cart_opt_ctrl)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_BUILD_TYPE ReleaseWithDebugInfo)

#find_package(Ceres REQUIRED)
#include_directories(${CERES_INCLUDE_DIRS})

find_package(GUROBI REQUIRED)
include_directories(${GUROBI_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
    rtt_ros
    kuka_lwr_fri
    diagnostic_msgs
    geometry_msgs
    std_msgs
    kdl_conversions
    eigen_conversions
    nav_msgs
    cmake_modules
)

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
   DEPENDS rtt
   CATKIN_DEPENDS rtt_ros diagnostic_msgs geometry_msgs std_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)
if($ENV{OROCOS_TARGET} STREQUAL "xenomai" )
  set_property(TARGET ${library_name} APPEND PROPERTY COMPILE_DEFINITIONS __XENOMAI__)
endif()

# Just for Hydro compatibility
list(REMOVE_ITEM catkin_LIBRARIES lwr_fri_generate_messages_cpp)
list(REMOVE_ITEM catkin_LIBRARIES lwr_fri_generate_messages_lisp)
list(REMOVE_ITEM catkin_LIBRARIES lwr_fri_generate_messages_py)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${kdl_conversions_LIBRARIES}
    ${Boost_LIBRARIES}
    ${GUROBI_LIBRARIES}
    #${CERES_LIBRARIES}
)

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})
orocos_generate_package(INCLUDE_DIRS include)
