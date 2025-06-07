cmake_minimum_required(VERSION 2.8.3)

project(mpi_spout)

# Set cpp 14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify additional locations of header files
include_directories(
  include
  SYSTEM
)

###########
## Build ##
###########

# Set build type and output location
set(CMAKE_BUILD_TYPE Release)	
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "lib")

# Make shared library 
add_library(mpi_spout SHARED
  src/mpi_spout.cpp
  src/jhcMpiSpout.cpp
  src/jhcFestTTS.cpp
)

# Required input libraries
target_link_libraries(mpi_spout
  libasound.so
  pthread
)
