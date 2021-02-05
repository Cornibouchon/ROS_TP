
add_executable(position_control src/position_control.cpp)
target_link_libraries(position_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(position_control_with_callback src/position_control_with_callback.cpp)
target_link_libraries(position_control_with_callback ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(pose_control src/pose_control.cpp)
target_link_libraries(pose_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cartesian_control src/cartesian_control.cpp)
target_link_libraries(cartesian_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Link to the marker package to be able to access the custom message
find_package(marker1)
catkin_package(CATKIN_DEPENDS message_runtime)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
