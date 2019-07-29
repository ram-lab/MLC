cmake_minimum_required(VERSION 2.8.3)
project(lego_loam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs
)

find_package(libpointmatcher REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)

message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")
message(STATUS "Using libpointmatcher include ${libpointmatcher_INCLUDE_DIRS}")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cloud_msgs
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
	${libpointmatcher_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

add_library(visualizer src/visualizer/Visualizer.cpp)
add_library(point_matcher_ros src/point_cloud.cpp)
add_library(camodocal src/camodocal/calib/PlanarHandEyeCalibration.cc)

add_executable(imageProjection src/imageProjection.cpp)
add_dependencies(imageProjection ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(featureAssociation src/featureAssociation.cpp)
add_dependencies(featureAssociation ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(featureAssociation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mapOptmization src/mapOptmization.cpp)
target_link_libraries(mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_executable(transformFusion src/transformFusion.cpp)
target_link_libraries(transformFusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(calib_refinement src/calib_refinement.cpp)
target_link_libraries(calib_refinement visualizer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} glog::glog)

add_executable(calib_refinement_icp src/calib_refinement_icp.cpp)
target_link_libraries(calib_refinement_icp visualizer ${catkin_LIBRARIES} ${PCL_LIBRARIES} 
	${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} glog::glog)
	
add_executable(calib_initialization src/calib_initialization.cpp)
target_link_libraries(calib_initialization camodocal ${catkin_LIBRARIES} ${PCL_LIBRARIES} 
	${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES} glog::glog)	

add_executable(icp_simple src/icp_simple.cpp)
target_link_libraries(icp_simple point_matcher_ros ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${libpointmatcher_LIBRARIES} glog::glog)	

add_executable(check_overlapping src/check_overlapping.cpp)
target_link_libraries(check_overlapping ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${libpointmatcher_LIBRARIES} point_matcher_ros glog::glog)

add_executable(check_overlapping_2 src/check_overlapping_2.cpp)
target_link_libraries(check_overlapping_2 ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${libpointmatcher_LIBRARIES} point_matcher_ros glog::glog)

add_executable(check_overlapping_3 src/check_overlapping_3.cpp)
target_link_libraries(check_overlapping_3 ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${libpointmatcher_LIBRARIES} point_matcher_ros glog::glog)

add_executable(check_overlapping_4 src/check_overlapping_4.cpp)
target_link_libraries(check_overlapping_4 ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${libpointmatcher_LIBRARIES} point_matcher_ros glog::glog)

add_executable(icp_simple_map src/icp_simple_map.cpp)
target_link_libraries(icp_simple_map point_matcher_ros ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${libpointmatcher_LIBRARIES} glog::glog)	



