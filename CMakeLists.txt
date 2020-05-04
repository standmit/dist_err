cmake_minimum_required(VERSION 2.8.3)
project(dist_err)

find_package(catkin REQUIRED COMPONENTS
    mavlink
    mavros
    pluginlib
    roscpp
)

catkin_package(
    LIBRARIES
        ${PROJECT_NAME}_plugins
    CATKIN_DEPENDS
        pluginlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_plugins src/plugins/dist_err.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${catkin_LIBRARIES})

install(
    TARGETS ${PROJECT_NAME}_plugins
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    FILES
        mavros_plugins.xml
        view_TF.rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
