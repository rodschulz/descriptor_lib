cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

# Project's name
project(DESCRIPTOR)

# Find required packages
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost 1.54.0 COMPONENTS unit_test_framework REQUIRED)

# Relevant PLC's folders
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Relevant BOOST's folders
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Set the build type for PCL
set(PCL_BUILD_TYPE Release)

# Add extra required definitions
add_definitions(${PCL_DEFINITIONS})
add_definitions("-Wall")

# Add subfolders to be compiled
add_subdirectory(clustering)
add_subdirectory(factories)
add_subdirectory(descriptor)
add_subdirectory(utils)
add_subdirectory(io)

# Define files for each target
file(GLOB DESCRIPTOR_SRC "main.cpp")
file(GLOB TEST_SRC
	"./test/*.h"
	"./test/*.cpp")
	
# Target Descriptor
add_executable(Descriptor ${DESCRIPTOR_SRC})
target_link_libraries(Descriptor 
			io
			utils
			clustering
			descriptor
			factories
			${PCL_LIBRARIES}
			${OpenCV_LIBS}
			${YAML_CPP_LIBRARIES})

# Target Test
add_executable(Test ${TEST_SRC})
set_target_properties(Test PROPERTIES COMPILE_DEFINITIONS "BOOST_TEST_DYN_LINK")
target_link_libraries(Test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
