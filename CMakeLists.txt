cmake_minimum_required(VERSION 3.5) # Specify the minimum required version of CMake
project(cppraplab) # Define the project name

# find_package(Boost REQUIRED) # Uncomment this if Boost is required for the project
find_package(Threads REQUIRED) # Find and include the Threads library (required for multithreading)

# Set the C++ standard to C++11 (or C++0x, which is an older name for C++11)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# Recursively find all header files in the "include" directory
file(GLOB_RECURSE ALL_HDRS "include/*.hpp")

# Recursively find all source files in the "source" directory
file(GLOB_RECURSE ALL_SRCS "source/*.cpp")

# Debugging message to print all header files (commented out)
# message("ALL_HDRS:", ${ALL_HDRS})

# Include directories for the project
include_directories(DIRECTORY 
  # include/${PROJECT_NAME} # Uncomment if you have a project-specific include directory
  include # Include the "include" directory
  # include/common # Uncomment if you have a "common" include directory
)

# Create a shared library from all the source files
add_library(${PROJECT_NAME} SHARED
  ${ALL_SRCS}
)

# Define the directory containing test files
set(test_cpp_dir "test/")

# Initialize an empty variable to hold test file paths
set(test_cpp_files "")

# Recursively find all test source files in the "test" directory
file(GLOB_RECURSE test_cpp_files "${test_cpp_dir}/*.cpp")

# Loop through each test file found
foreach(test_cpp_file ${test_cpp_files})
  # Extract the base name (without extension) of the test file
  get_filename_component(test_cpp_name ${test_cpp_file} NAME_WE)

  # Add an executable for each test file
  # The executable will be named after the test file (without extension)
  # ADD_EXECUTABLE(${test_cpp_name} ${ALL_SRCS} ${test_cpp_dir}/${test_cpp_name}.cpp) # Uncomment if you want to link all source files to the test executable
  ADD_EXECUTABLE(${test_cpp_name} ${test_cpp_dir}/${test_cpp_name}.cpp)

  # Link the test executable with the project library and threading library
  TARGET_LINK_LIBRARIES(${test_cpp_name} 
    ${PROJECT_NAME} # Link the shared library created earlier
    ${CMAKE_THREAD_LIBS_INIT} # Link the threading library
  )
endforeach(test_cpp_file ${test_cpp_files}) # End of the loop
