cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )

include_directories(${GAZEBO_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)

link_directories(${GAZEBO_LIBRARY_DIRS}  ${CMAKE_CURRENT_BINARY_DIR}/msgs ${CMAKE_CURRENT_BINARY_DIR}/tarotPB/msgs/build)
add_subdirectory(msgs)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(Octorotor_Env SHARED octorotor_env_plugin.cc)
add_library(Tarot_Motor_Model SHARED tarot_motor_model.cc)
add_library(S1000_Motor_Model SHARED S1000_motor_model.cc)


target_link_libraries(Octorotor_Env tarotPB ${GAZEBO_LIBRARIES})
target_link_libraries(Tarot_Motor_Model tarotPB ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(S1000_Motor_Model tarotPB ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

add_dependencies(Tarot_Motor_Model tarotPB)
add_dependencies(S1000_Motor_Model tarotPB)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
