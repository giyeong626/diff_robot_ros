set(PROJECT_NAME 2ediff_rob_bringup)
project(${PROJECT_NAME})

cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        include ${catkin_INCLUDE_DIRS}
)

add_executable(2ediffrob main.cc)
target_link_libraries(
        2ediffrob
        ${catkin_LIBRARIES}
        )
add_dependencies(2ediffrob 2Ediff_generate_messages_cpp)
