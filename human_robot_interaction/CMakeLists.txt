cmake_minimum_required(VERSION 2.8.3)
project(human_robot_interaction)

find_package(catkin REQUIRED)
catkin_package(DEPENDS)

# catkin_package()


install(DIRECTORY demo
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
