cmake_minimum_required(VERSION 2.8.3)
project(farmbeast_navigation)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roslib geometry_msgs roscpp rospy std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(navigation_compass2019_test src/navigation_compass2019_test.cpp)
target_link_libraries(navigation_compass2019_test ${catkin_LIBRARIES})

add_executable(navigation_obracanje_task2_2019_v2 src/navigation_obracanje_task2_2019_v2.cpp)
target_link_libraries(navigation_obracanje_task2_2019_v2 ${catkin_LIBRARIES})


add_executable(navigation_pregled_stopinj src/navigation_pregled_stopinj.cpp)
target_link_libraries(navigation_pregled_stopinj ${catkin_LIBRARIES})

add_executable(navigation_obracanje_task2_2019 src/navigation_obracanje_task2_2019.cpp)
target_link_libraries(navigation_obracanje_task2_2019 ${catkin_LIBRARIES})

add_executable(navigation_orientacija src/navigation_orientacija.cpp)
target_link_libraries(navigation_orientacija ${catkin_LIBRARIES})

add_executable(navigation_Task1 src/navigation_Task1.cpp)
target_link_libraries(navigation_Task1 ${catkin_LIBRARIES})

add_executable(navigation_compass src/navigation_compass.cpp)
target_link_libraries(navigation_compass ${catkin_LIBRARIES})

add_executable(navigation_compass2 src/navigation_compass2.cpp)
target_link_libraries(navigation_compass2 ${catkin_LIBRARIES})

add_executable(navigation_rowmaster src/navigation_rowmaster.cpp)
target_link_libraries(navigation_rowmaster ${catkin_LIBRARIES})

add_executable(navigation_koncna src/navigation_koncna.cpp)
target_link_libraries(navigation_koncna ${catkin_LIBRARIES})

add_executable(navigation_rm_ackermann src/navigation_rm_ackermann.cpp)
target_link_libraries(navigation_rm_ackermann ${catkin_LIBRARIES})

add_executable(navigation_rm_povpr src/navigation_rm_povpr.cpp)
target_link_libraries(navigation_rm_povpr ${catkin_LIBRARIES})

add_executable(navigation_rm_combined src/navigation_rm_combined.cpp)
target_link_libraries(navigation_rm_combined ${catkin_LIBRARIES})

add_executable(navigation_rm_combined2 src/navigation_rm_combined2.cpp)
target_link_libraries(navigation_rm_combined2 ${catkin_LIBRARIES})

add_executable(navigation_rm_obracanje src/navigation_rm_obracanje.cpp)
target_link_libraries(navigation_rm_obracanje ${catkin_LIBRARIES})

add_executable(farmbeast_joze_rm src/farmbeast_joze_rm.cpp)
target_link_libraries(farmbeast_joze_rm ${catkin_LIBRARIES})

add_executable(farmbeast_peterb_rm src/farmbeast_peterb_rm.cpp)
target_link_libraries(farmbeast_peterb_rm ${catkin_LIBRARIES})

add_executable(farmbeast_povpr_rm src/farmbeast_povpr_rm.cpp)
target_link_libraries(farmbeast_povpr_rm ${catkin_LIBRARIES})

add_executable(test_urban src/testUrban.cpp)
target_link_libraries(test_urban ${catkin_LIBRARIES})

add_executable(imu_node src/imu.cpp)
target_link_libraries(imu_node ${catkin_LIBRARIES})

add_executable(counting_rows src/counting_rows.cpp)
target_link_libraries(counting_rows ${catkin_LIBRARIES})

add_executable(navigation_2022 src/navigation_2022.cpp)
target_link_libraries(navigation_2022 ${catkin_LIBRARIES})

