cmake_minimum_required(VERSION 3.5)
project(cppraplab)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs highgui imgproc)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# Recursively find all header files in the "include" directory
file(GLOB_RECURSE ALL_HDRS "include/*.hpp")

# Recursively find all source files in the "source" directory
file(GLOB_RECURSE ALL_SRCS "source/*.cpp")

# Include directories for the project
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)
# Create a shared library from all the source files
add_library(${PROJECT_NAME} SHARED
  ${ALL_SRCS}
)

target_link_libraries(${PROJECT_NAME}
  Threads::Threads
  ${OpenCV_LIBS}
)

# Define the directory containing test files
set(test_cpp_dir "test/")

# Initialize an empty variable to hold test file paths
set(test_cpp_files "")

# Recursively find all test source files in the "test" directory
file(GLOB_RECURSE test_cpp_files "${test_cpp_dir}/*.cpp")

# Loop through each test file found
foreach(test_cpp_file ${test_cpp_files})
  get_filename_component(test_cpp_name ${test_cpp_file} NAME_WE)

  add_executable(${test_cpp_name} ${test_cpp_file})

  target_link_libraries(${test_cpp_name} 
    ${PROJECT_NAME}
    Threads::Threads
    ${OpenCV_LIBS}
  )
endforeach()

add_executable(
  animation_astar animation/animation_astar.cpp
)
target_link_libraries(animation_astar
  ${PROJECT_NAME}
  Threads::Threads
  ${OpenCV_LIBS}
)

add_executable(
  animation_astar_time animation/animation_astar_time.cpp
)
target_link_libraries(animation_astar_time
  ${PROJECT_NAME}
  Threads::Threads
  ${OpenCV_LIBS}
)

message(STATUS "OpenCV Include Directories: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV Libraries: ${OpenCV_LIBS}")