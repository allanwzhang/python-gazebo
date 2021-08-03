cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )

include_directories(${GAZEBO_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)

link_directories(${GAZEBO_LIBRARY_DIRS}  ${CMAKE_CURRENT_BINARY_DIR}/msgs ${CMAKE_CURRENT_BINARY_DIR}/tarotPB/msgs/build)
add_subdirectory(msgs)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(esc_plugin SHARED esc_plugin.cc)
add_library(tarot_env SHARED tarot_env_plugin.cc)


target_link_libraries(esc_plugin tarotPB ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(tarot_env tarotPB ${GAZEBO_LIBRARIES})

add_dependencies(esc_plugin tarotPB)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


