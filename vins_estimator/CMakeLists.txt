cmake_minimum_required(VERSION 2.8.3)
project(vins_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++1z")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
  camera_model
  )

find_package(Gflags)
find_package(Glog)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(TBB REQUIRED)

#find_package( Sophus REQUIRED )
#include_directories( ${Sophus_INCLUDE_DIRS} )
include_directories(thirdparty/Sophus)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})


catkin_package(CATKIN_DEPENDS camera_model)



add_executable(vins_estimator
  src/estimator_node.cpp
  src/parameters.cpp
  src/estimator.cpp

  src/featureTrack/feature_tracker.cpp
  src/Frontend/frontend.cpp
  src/feature_manager.cpp

  src/factor/pose_local_parameterization.cpp
  src/factor/projection_factor.cpp
  src/factor/projection_td_factor.cpp
  src/factor/marginalization_factor.cpp
  src/factor/SE3AbsolutatePoseFactor.cpp

  src/utility/utility.cpp
  src/utility/visualization.cpp
  src/utility/CameraPoseVisualization.cpp
  src/initial/solve_5pts.cpp
  src/initial/initial_sfm.cpp
  src/initial/initial_aligment.cpp
  src/initial/initial_ex_rotation.cpp
  )


add_dependencies(vins_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(vins_estimator
  ${catkin_LIBRARIES}
  ${TBB_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES})

# add_executable(
#   pose_graph3d
#   src/pose_graph3d.cpp
#   src/factor/FrameParameterization.cpp
#   src/factor/SE3RelativtePoseFactor.cpp
# )

# target_link_libraries(
#   pose_graph3d
#   ${CERES_LIBRARIES}
#   ${GLOG_LIBRARIES}
#   ${GFLAGS_LIBRARIES}
# )


