cmake_minimum_required(VERSION 2.8.3)
project(corner_event_detector)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin_simple REQUIRED)
catkin_simple()

list(APPEND SOURCE_FILES
  src/detector.cpp
  src/harris_detector.cpp
  src/fast_detector.cpp
  src/distinct_queue.cpp
  src/fixed_distinct_queue.cpp
)

list(APPEND HEADER_FILES
  include/corner_event_detector/detector.h
  include/corner_event_detector/harris_detector.h
  include/corner_event_detector/fast_detector.h
  include/corner_event_detector/local_event_queues.h
  include/corner_event_detector/distinct_queue.h
  include/corner_event_detector/fixed_distinct_queue.h
)

cs_add_executable(detector ${SOURCE_FILES} src/node.cpp)

cs_export()
