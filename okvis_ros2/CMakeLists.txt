cmake_minimum_required(VERSION 3.8...3.30)
project(okvis_ros2 VERSION 2.0.0 LANGUAGES CXX)

include_directories(include/)
MESSAGE(STATUS "Building ROS2 wrapper")

#LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
# ROS2 DEPENDENCIES
FIND_PACKAGE(ament_cmake REQUIRED)
FIND_PACKAGE(ament_cmake_python REQUIRED)
FIND_PACKAGE(builtin_interfaces REQUIRED)
FIND_PACKAGE(eigen3_cmake_module REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(cv_bridge REQUIRED)  
FIND_PACKAGE(sensor_msgs REQUIRED)
FIND_PACKAGE(visualization_msgs REQUIRED)
FIND_PACKAGE(nav_msgs REQUIRED)
find_package(Boost REQUIRED)

find_package(PCL REQUIRED QUIET COMPONENTS common)
include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(pcl_msgs REQUIRED)
FIND_PACKAGE(pcl_conversions REQUIRED)
FIND_PACKAGE(rclpy REQUIRED)
FIND_PACKAGE(rclcpp REQUIRED)
FIND_PACKAGE(rclcpp_components REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
FIND_PACKAGE(tf2 REQUIRED)
FIND_PACKAGE(tf2_ros REQUIRED)
FIND_PACKAGE(image_transport REQUIRED)
FIND_PACKAGE(message_filters REQUIRED)
FIND_PACKAGE(cv_bridge REQUIRED)

# INCLUDES AND LIB DIRECTORIES
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Eigen3_INCLUDE_DIRS})
include_directories( include ${PCL_INCLUDE_DIRS} )

# okvis::Ros2 library
set(LIB_NAME okvis_ros2)
add_library(${LIB_NAME}
  src/Subscriber.cpp
  src/RosbagReader.cpp
  src/Publisher.cpp
  src/RePublisher.cpp
)
target_include_directories(${LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${GLOG_INCLUDE_DIRS}
)
target_link_libraries(${LIB_NAME}
    ${OpenCV_LIBS}
    Ceres::ceres
    Eigen3::Eigen
    okvis::Common
    okvis::MultisensorProcessing
    ${GLOG_LIBRARIES}
    ${Boost_LIBRARIES} 
    ${PCL_LIBRARIES}
)
target_compile_features(${LIB_NAME} PUBLIC cxx_std_${OKVIS_CXX_STANDARD})
target_compile_options(${LIB_NAME}
  PUBLIC
    ${OKVIS_PUBLIC_CXX_FLAGS}
  PRIVATE
    ${OKVIS_PRIVATE_CXX_FLAGS}
)
if(OPENMP_FOUND)
  target_compile_definitions(${LIB_NAME} PRIVATE USE_OPENMP)
endif()
add_library(okvis::Ros2 ALIAS ${LIB_NAME})

# TARGET DEPENDENCIES
AMENT_TARGET_DEPENDENCIES(
  ${LIB_NAME}
  PCL
  ament_cmake
  rclcpp
  rclcpp_components
  rosbag2_cpp
  rosbag2_storage
  rosidl_default_runtime
  eigen3_cmake_module
  Eigen3
  sensor_msgs
  visualization_msgs
  nav_msgs
  pcl_conversions
  message_filters
  cv_bridge
  image_transport
  tf2
  tf2_ros
)

# EXPORT DEPENDENCIES
AMENT_EXPORT_DEPENDENCIES(
  PCL
  ament_cmake
  rclcpp
  rclcpp_components
  rosbag2_cpp
  rosbag2_storage
  rosidl_default_runtime
  eigen3_cmake_module
  Eigen3
  sensor_msgs
  visualization_msgs
  nav_msgs
  pcl_conversions
  message_filters
  cv_bridge
  image_transport
  tf2
  tf2_ros
)

install(TARGETS ${LIB_NAME} EXPORT okvisTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/okvis DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")

# OKVIS nodes

ADD_EXECUTABLE(okvis_node_synchronous src/okvis_node_synchronous.cpp)
TARGET_LINK_LIBRARIES(
  okvis_node_synchronous
  okvis::Ros2
  ${OpenCV_LIBS}
  ${SUITESPARSE_LIBRARIES}
  ${Boost_LIBRARIES} 
  ${PCL_LIBRARIES}
)
INSTALL(TARGETS okvis_node_synchronous DESTINATION lib/${PROJECT_NAME})

ADD_EXECUTABLE(okvis_node_subscriber src/okvis_node_subscriber.cpp)
TARGET_LINK_LIBRARIES(
  okvis_node_subscriber
  okvis::Ros2
  ${OpenCV_LIBS}
  ${SUITESPARSE_LIBRARIES}
  ${Boost_LIBRARIES} 
  ${PCL_LIBRARIES}
)
INSTALL(TARGETS okvis_node_subscriber DESTINATION lib/${PROJECT_NAME})

if(HAVE_LIBREALSENSE)
  ADD_EXECUTABLE(okvis_node_realsense src/okvis_node_realsense.cpp)
  TARGET_LINK_LIBRARIES(
    okvis_node_realsense
    okvis::Ros2
    ${OpenCV_LIBS}
    ${SUITESPARSE_LIBRARIES}
    ${Boost_LIBRARIES} 
    ${PCL_LIBRARIES}
  )
  INSTALL(TARGETS okvis_node_realsense DESTINATION lib/${PROJECT_NAME})
  
  ADD_EXECUTABLE(okvis_node_realsense_publisher src/okvis_node_realsense_publisher.cpp)
  TARGET_LINK_LIBRARIES(
    okvis_node_realsense_publisher
    okvis::Ros2
    ${OpenCV_LIBS}
    ${SUITESPARSE_LIBRARIES}
    ${Boost_LIBRARIES} 
    ${PCL_LIBRARIES}
  )
  INSTALL(TARGETS okvis_node_realsense_publisher DESTINATION lib/${PROJECT_NAME})
endif()

# INSTALL LAUNCH FILES
  INSTALL(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
  INSTALL(DIRECTORY ../resources DESTINATION share/${PROJECT_NAME}/)
  INSTALL(DIRECTORY ../config DESTINATION share/${PROJECT_NAME}/)

# FORM ROS2 Package
AMENT_PACKAGE()