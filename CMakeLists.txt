cmake_minimum_required(VERSION 3.0.2)
project(local_excavation)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_DEPENDENCIES
        grid_map_ros
        grid_map_msgs
        grid_map_cv
        grid_map_sdf
        grid_map_core
        roscpp
        rospy
        m545_msgs
        pcl_ros
        kindr_ros
        signal_logger
        tinyxml_tools
        curves
        loco_m545
        excavation_mapping
        geometry_msgs
        tf2_ros
        tf2
        )

set(CATKIN_DEPENDS
        ${CATKIN_DEPENDENCIES}
        )


find_package(catkin REQUIRED
        COMPONENTS
            ${CATKIN_DEPENDENCIES}
        )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(LAPACK REQUIRED)
find_package(OpenCV)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES local_excavation
        CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
        DEPENDS Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
        src/LocalPlanner.cpp)


target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)


## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    #  find_package(rostest REQUIRED)
    #  add_rostest_gtest(${PROJECT_NAME}_rostest test/test_local_excavation.launch
    #         test/test_local_excavation.cpp)
    #  target_link_libraries(${PROJECT_NAME}_rostest
    #         ${catkin_LIBRARIES})
    # normal catkin test
    catkin_add_gtest(${PROJECT_NAME}_test test/test_local_excavation.cpp)
    if (TARGET ${PROJECT_NAME}_test)
        target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    endif ()
endif ()

# Add folders to be run by python nosetests