cmake_minimum_required(VERSION 2.8.3)
project(iiwa_arm)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
		message_generation
		moveit_core
		moveit_ros_planning
		moveit_ros_planning_interface
		pluginlib
		geometric_shapes
		moveit_visual_tools
		roscpp
		roslint
		rospy
	)
find_package(Eigen3 REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   CalculateIK.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    geometry_msgs
    trajectory_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_executable(trajectory_sampler src/trajectory_sampler.cpp)

add_dependencies(trajectory_sampler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(trajectory_sampler
   ${catkin_LIBRARIES}
 )

 ## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
    scripts/random_spawn.py
    scripts/safe_spawner.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   config/spawn_locations.yaml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
