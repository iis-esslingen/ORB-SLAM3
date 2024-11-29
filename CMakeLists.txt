cmake_minimum_required(VERSION 2.8)
project(orb_slam3_ros)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check for C++14 or C++0x support.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  add_definitions(-DCOMPILEDWITHC11)

  message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)

  message(STATUS "Using flag -std=c++0x.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

find_package(OpenCV 4.2)

if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV 4.2 not found.")
endif()

message(STATUS "OpenCV version: " ${OpenCV_VERSION})

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  message_filters
  roscpp
  rospy
  tf
  tf2
  message_generation
)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

add_service_files(
  FILES
  SaveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs dynamic_reconfigure message_runtime
  LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
)

include_directories(
  "include"
  "orb_slam3"
  "orb_slam3/include"
  "orb_slam3/include/CameraModels"
  "orb_slam3/Thirdparty/Sophus"
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory("orb_slam3/Thirdparty/DBoW2")
add_subdirectory("orb_slam3/Thirdparty/g2o")

add_library(${PROJECT_NAME} SHARED
  "orb_slam3/src/System.cc"
  "orb_slam3/src/Tracking.cc"
  "orb_slam3/src/LocalMapping.cc"
  "orb_slam3/src/LoopClosing.cc"
  "orb_slam3/src/ORBextractor.cc"
  "orb_slam3/src/ORBmatcher.cc"
  "orb_slam3/src/FrameDrawer.cc"
  "orb_slam3/src/Converter.cc"
  "orb_slam3/src/MapPoint.cc"
  "orb_slam3/src/KeyFrame.cc"
  "orb_slam3/src/Atlas.cc"
  "orb_slam3/src/Map.cc"
  "orb_slam3/src/MapDrawer.cc"
  "orb_slam3/src/Optimizer.cc"
  "orb_slam3/src/Frame.cc"
  "orb_slam3/src/KeyFrameDatabase.cc"
  "orb_slam3/src/Sim3Solver.cc"
  "orb_slam3/src/Viewer.cc"
  "orb_slam3/src/ImuTypes.cc"
  "orb_slam3/src/G2oTypes.cc"
  "orb_slam3/src/CameraModels/Pinhole.cpp"
  "orb_slam3/src/CameraModels/KannalaBrandt8.cpp"
  "orb_slam3/src/OptimizableTypes.cpp"
  "orb_slam3/src/MLPnPsolver.cpp"
  "orb_slam3/src/GeometricTools.cc"
  "orb_slam3/src/TwoViewReconstruction.cc"
  "orb_slam3/src/Config.cc"
  "orb_slam3/src/Settings.cc"
  "orb_slam3/include/System.h"
  "orb_slam3/include/Tracking.h"
  "orb_slam3/include/LocalMapping.h"
  "orb_slam3/include/LoopClosing.h"
  "orb_slam3/include/ORBextractor.h"
  "orb_slam3/include/ORBmatcher.h"
  "orb_slam3/include/FrameDrawer.h"
  "orb_slam3/include/Converter.h"
  "orb_slam3/include/MapPoint.h"
  "orb_slam3/include/KeyFrame.h"
  "orb_slam3/include/Atlas.h"
  "orb_slam3/include/Map.h"
  "orb_slam3/include/MapDrawer.h"
  "orb_slam3/include/Optimizer.h"
  "orb_slam3/include/Frame.h"
  "orb_slam3/include/KeyFrameDatabase.h"
  "orb_slam3/include/Sim3Solver.h"
  "orb_slam3/include/Viewer.h"
  "orb_slam3/include/ImuTypes.h"
  "orb_slam3/include/G2oTypes.h"
  "orb_slam3/include/CameraModels/GeometricCamera.h"
  "orb_slam3/include/CameraModels/Pinhole.h"
  "orb_slam3/include/CameraModels/KannalaBrandt8.h"
  "orb_slam3/include/OptimizableTypes.h"
  "orb_slam3/include/MLPnPsolver.h"
  "orb_slam3/include/GeometricTools.h"
  "orb_slam3/include/TwoViewReconstruction.h"
  "orb_slam3/include/SerializationUtils.h"
  "orb_slam3/include/Config.h"
  "orb_slam3/include/Settings.h"
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  DBoW2
  g2o
  -lboost_system
  -lboost_serialization
  -lcrypto
)

add_executable(ros_mono
  "src/ros_mono.cc"
  "src/common.cc"
)
target_link_libraries(ros_mono
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(ros_mono_inertial
  "src/ros_mono_inertial.cc"
  "src/common.cc"
)
target_link_libraries(ros_mono_inertial
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(ros_stereo
  "src/ros_stereo.cc"
  "src/common.cc"
)
target_link_libraries(ros_stereo
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(ros_stereo_inertial
  "src/ros_stereo_inertial.cc"
  "src/common.cc"
)
target_link_libraries(ros_stereo_inertial
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(ros_rgbd
  "src/ros_rgbd.cc"
  "src/common.cc"
)
target_link_libraries(ros_rgbd
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(ros_rgbd_inertial
  "src/ros_rgbd_inertial.cc"
  "src/common.cc"
)
target_link_libraries(ros_rgbd_inertial
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)