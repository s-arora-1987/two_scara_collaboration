cmake_minimum_required(VERSION 2.8.3)
project(two_scara_collaboration)
add_compile_options(-std=c++11)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next line to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)

#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
#generate_dynamic_reconfigure_options(
#  cfg/wrench_dyn_reconfig.cfg
#) 

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
# cs_add_executable(example src/example.cpp)
cs_add_executable(cylinder_blocks_spawner src/cylinder_blocks_spawner.cpp)
cs_add_executable(cylinder_blocks_poses_publisher src/cylinder_blocks_poses_publisher.cpp)
cs_add_executable(scara_gripper_action_server src/scara_gripper_action_server.cpp)
cs_add_executable(scara_joint_controller src/scara_joint_controller.cpp)
cs_add_executable(cylinder_active_pool src/cylinder_active_pool.cpp)
cs_add_executable(scara_upper_boundary_maintainer src/scara_upper_boundary_maintainer.cpp)
cs_add_executable(scara_left_motion_planner src/scara_left_motion_planner.cpp)
cs_add_executable(scara_right_motion_planner src/scara_right_motion_planner.cpp)
cs_add_executable(claim_n_track_onion src/claim_n_track_onion.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib)

# make sure configure headers are built before any node using them
#add_dependencies(cylinder_blocks_poses_publisher two_scara_collaboration_gencfg)

cs_install()
cs_export()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)


