cmake_minimum_required(VERSION 3.1.3)
project(moveit_packages)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
    marker1
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  interactivity/include
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_subdirectory(moveit_cpp)

add_subdirectory(move_group_interface)
add_subdirectory(controllers)
# add_subdirectory(controller_configuration)
# add_subdirectory(hand_eye_calibration)
# add_subdirectory(interactivity)
# add_subdirectory(kinematics)
# add_subdirectory(motion_planning_api)
# add_subdirectory(motion_planning_pipeline)

# add_subdirect# ory(move_group_python_interface)
# add_subdirectory(perception_pipeline)
# add_subdirectory(pick_place)
# add_subdirectory(planning)
# add_subdirectory(planning_scene)
# add_subdirectory(planning_scene_ros_api)
# add_subdirectory(robot_model_and_robot_state)
# add_subdirectory(state_display)
# add_subdirectory(subframes)
# add_subdirectory(tests)
# add_subdirectory(trajopt_planner)
# add_subdirectory(creating_moveit_plugins/lerp_motion_planner)
# add_subdirectory(collision_environments)
# add_subdirectory(visualizing_collisions)
# add_subdirectory(bullet_collision_checker)

