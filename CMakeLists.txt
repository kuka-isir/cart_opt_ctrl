cmake_minimum_required(VERSION 2.8.3)
project(cart_opt_ctrl)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    rtt_ros
    kdl_conversions
    eigen_conversions
    cmake_modules
    qpOASES
)

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)

orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)



target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${kdl_conversions_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
)

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})
orocos_generate_package(INCLUDE_DIRS include)
