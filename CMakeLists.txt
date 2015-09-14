cmake_minimum_required(VERSION 2.8.3)
project(cart_opt_ctrl)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_BUILD_TYPE Release)

#find_package(Ceres)
#if(Ceres_FOUND)
#    include_directories(${CERES_INCLUDE_DIRS})
#endif()

find_package(qpOASES)
if(qpOASES_FOUND)
    include_directories(${qpOASES_INCLUDE_DIRS})
endif()

find_package(GUROBI)
if(GUROBI_FOUND)
    include_directories(${GUROBI_INCLUDE_DIRS})
endif()

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
#find_package(Boost REQUIRED system)

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
if($ENV{OROCOS_TARGET} STREQUAL "xenomai")
  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS __XENOMAI__)
endif()

# Just for Hydro compatibility
list(REMOVE_ITEM catkin_LIBRARIES lwr_fri_generate_messages_cpp)
list(REMOVE_ITEM catkin_LIBRARIES lwr_fri_generate_messages_lisp)
list(REMOVE_ITEM catkin_LIBRARIES lwr_fri_generate_messages_py)

if(GUROBI_FOUND)
  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_GUROBI)
  message(STATUS "Using Gurobi")
endif()

if(CERES_FOUND)
  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_CERES)
  message(STATUS "Using Ceres")
endif()

if(qpOASES_FOUND)
  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_QPOASES)
  message(STATUS "Using qpOASES")
endif()


target_link_libraries(${PROJECT_NAME}
    #${Xenomai_LIBRARIES}
    ${catkin_LIBRARIES}
    
    ${orocos_kdl_LIBRARIES}
    ${kdl_conversions_LIBRARIES}
    #${Boost_LIBRARIES}
    ${GUROBI_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    #${Xenomai_LIBRARIES}
    ${CERES_LIBRARIES}
    ${qpOASES_LIBRARIES}
)

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})
orocos_generate_package(INCLUDE_DIRS include)
