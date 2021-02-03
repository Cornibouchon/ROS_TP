
cmake_minimum_required(VERSION 2.8.3)
project(moveit_python)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation kortex_driver)

## Declare a catkin package
catkin_package()
catkin_python_setup()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)


# Install

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS 
                      src/move_it/gripx_python.py 
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
