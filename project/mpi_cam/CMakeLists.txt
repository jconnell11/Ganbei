cmake_minimum_required(VERSION 2.8.3)

project(mpi_cam)

# Set cpp 14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configure OpenCV 
find_package(OpenCV REQUIRED)

# Specify additional locations of header files
include_directories(
  include
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
)

###########
## Build ##
###########

# Set build type and output location
set(CMAKE_BUILD_TYPE Release)	
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "lib")

# Make shared library
add_library(mpi_cam SHARED
  src/mpi_cam.cpp
  src/jhcMpiCam.cpp
)

# Required input libraries for shared lib
target_link_libraries(mpi_cam
  pthread
  ${OpenCV_LIBS} 
)

