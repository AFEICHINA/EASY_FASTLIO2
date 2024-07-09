cmake_minimum_required(VERSION 3.8)
project(fastlio2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS Yes)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=2)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  tf2_ros
  image_transport
  livox_ros_driver
)

find_package(OpenMP QUIET)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(yaml-cpp REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
set(SRC_LIST src/map_builder/commons.cpp
             src/map_builder/ieskf.cpp
             src/map_builder/imu_processor.cpp
             src/map_builder/ikd_Tree.cpp
             src/map_builder/lidar_processor.cpp
             src/map_builder/map_builder.cpp
             src/utils.cpp)
#
catkin_package(

)

add_executable(lio_node src/lio_node.cpp ${SRC_LIST})
target_link_libraries(lio_node
  yaml-cpp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# install(TARGETS lio_node DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

