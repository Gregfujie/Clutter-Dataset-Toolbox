cmake_minimum_required(VERSION 2.8.3)
project(virtual_dataset)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  controller_manager
  joint_state_publisher
  robot_state_publisher
  rospy
  rviz
  std_msgs
  sensor_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES gazebo_ros_ouster_laser gazebo_ros_ouster_gpu_laser imu9dof
  DEPENDS
    roscpp
    gazebo_ros
    controller_manager
    joint_state_publisher
    robot_state_publisher
    rospy
    rviz
    std_msgs
    sensor_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(include ${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(car_steer_plugin src/car_steer.cpp src/cubicSpline.cpp src/PID.cpp)
target_link_libraries(car_steer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(rot_motion_plugin src/rotation_motion.cpp)
target_link_libraries(rot_motion_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_ouster_laser src/GazeboRosOusterLaser.cpp)
target_link_libraries(gazebo_ros_ouster_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

add_library(gazebo_ros_ouster_gpu_laser src/GazeboRosOusterLaser.cpp)
target_link_libraries(gazebo_ros_ouster_gpu_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  GpuRayPlugin
)

add_library(imu9dof src/Imu9dof.cpp)
target_link_libraries(imu9dof
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

target_compile_definitions(gazebo_ros_ouster_gpu_laser PRIVATE GAZEBO_GPU_RAY=1)

install(TARGETS gazebo_ros_ouster_laser gazebo_ros_ouster_gpu_laser imu9dof
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)