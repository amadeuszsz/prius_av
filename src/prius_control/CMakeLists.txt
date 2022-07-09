cmake_minimum_required(VERSION 3.0.2)
project(prius_control)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        message_generation
        message_filters
        image_transport
        cv_bridge
        prius_msgs
        dynamic_reconfigure
        )

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/pid.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES prius_control
  CATKIN_DEPENDS roscpp
        std_msgs
        sensor_msgs
        message_filters
        image_transport
        cv_bridge
        prius_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

add_executable(prius_control_node src/control.cpp include/prius_control/control.h)
add_executable(prius_state_node src/state.cpp include/prius_control/state.h)
add_executable(prius_collector_node src/collector.cpp include/prius_control/collector.h)
add_executable(prius_velocity_pid_node src/velocity_pid.cpp include/prius_control/velocity_pid.h src/pid.cpp include/prius_control/pid.h)
add_executable(prius_velocity_pid_tune_node src/velocity_pid_tune.cpp include/prius_control/velocity_pid_tune.h src/pid.cpp include/prius_control/pid.h)
add_executable(prius_visualizer_node src/visualizer.cpp include/prius_control/visualizer.h)

add_dependencies(prius_velocity_pid_tune_node ${PROJECT_NAME}_gencfg)

target_link_libraries(prius_control_node
        ${catkin_LIBRARIES}
        )
target_link_libraries(prius_state_node
        ${catkin_LIBRARIES}
        )
target_link_libraries(prius_collector_node
         ${catkin_LIBRARIES}
         ${OpenCV_LIBRARIES}
         )
target_link_libraries(prius_velocity_pid_node
        ${catkin_LIBRARIES}
        )
target_link_libraries(prius_velocity_pid_tune_node
        ${catkin_LIBRARIES}
        )
target_link_libraries(prius_visualizer_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )
