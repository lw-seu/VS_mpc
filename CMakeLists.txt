cmake_minimum_required(VERSION 3.0.2)
project(visual_servo_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
# #-DEIGEN_USE_MKL_ALL")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  cv_bridge
  image_transport
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/mpc_controller)
find_package(ACADO REQUIRED)
# At least Eigen 3.4 is required
find_package(Eigen3 REQUIRED VERSION 3.4)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${ACADO_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/tools
  ${PROJECT_SOURCE_DIR}/mpc_controller/quadrotor_mpc_export/
  ${PROJECT_SOURCE_DIR}/external_packages/qpoases
  ${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC
  ${PROJECT_SOURCE_DIR}/external_packages/qpoases/INCLUDE
)

add_library(mpc_solver_lib
  mpc_controller/quadrotor_mpc_export/acado_auxiliary_functions.c 
  mpc_controller/quadrotor_mpc_export/acado_integrator.c 
  mpc_controller/quadrotor_mpc_export/acado_qpoases_interface.cpp 
  mpc_controller/quadrotor_mpc_export/acado_solver.c
  external_packages/qpoases/SRC/Bounds.cpp 
  external_packages/qpoases/SRC/Constraints.cpp 
  external_packages/qpoases/SRC/CyclingManager.cpp 
  external_packages/qpoases/SRC/Indexlist.cpp 
  external_packages/qpoases/SRC/MessageHandling.cpp 
  external_packages/qpoases/SRC/QProblem.cpp 
  external_packages/qpoases/SRC/QProblemB.cpp 
  external_packages/qpoases/SRC/SubjectTo.cpp 
  external_packages/qpoases/SRC/Utils.cpp
  external_packages/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp)

add_library(mpc_wrapper_lib
  src/mpc_wrapper.cpp)
target_link_libraries(mpc_wrapper_lib mpc_solver_lib ${ACADO_SHARED_LIBRARIES} )

add_library(visual_servo_lib
  src/visual_servo.cpp)
target_link_libraries(visual_servo_lib ${catkin_LIBRARIES})

add_library(mpc_ros_application_lib
  src/mpc_ros_application.cpp)
target_link_libraries(mpc_ros_application_lib mpc_wrapper_lib)

add_library(disturbance_observer_lib
  src/disturbance_observer.cpp)
target_link_libraries(disturbance_observer_lib ${catkin_LIBRARIES})

add_library(mpc_inspection_lib
  src/pipeline_inspection.cpp)
target_link_libraries(mpc_inspection_lib mpc_ros_application_lib visual_servo_lib disturbance_observer_lib)

add_library(px4_interface_lib
  src/px4_interface.cpp)
target_link_libraries(px4_interface_lib mpc_ros_application_lib ${catkin_LIBRARIES})

add_executable(inspection_mpc src/inspection_node.cpp)
target_link_libraries(inspection_mpc ${catkin_LIBRARIES} 
  mpc_solver_lib
  mpc_wrapper_lib
  mpc_ros_application_lib
  mpc_inspection_lib
  px4_interface_lib) 