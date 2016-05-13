cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Project's name
project(DESCRIPTOR)

# Find required packages
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4.9 REQUIRED)
find_package(yaml-cpp 0.5.2 REQUIRED)
find_package(Boost 1.46 COMPONENTS unit_test_framework REQUIRED)
find_package(OpenMP REQUIRED)

# Print information related to OpenCV version
message(STATUS "Found OpenCV ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS= ${OpenCV_INCLUDE_DIRS}")

# Relevant OpenCV's folders
include_directories(${OpenCV_INCLUDE_DIRS})

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

# Add sub-folders to be compiled
add_subdirectory(clustering)
add_subdirectory(descriptor)
add_subdirectory(factories)
add_subdirectory(io)
add_subdirectory(metrics)
add_subdirectory(utils)

# Define files for each target
file(GLOB DESCRIPTOR_SRC "Descriptor.cpp")
file(GLOB TEST_SRC 
	"Test.cpp"
	"./test/*.hpp"
	"./test/*.cpp")

# Target Test
add_executable(Test ${TEST_SRC})
set_target_properties(Test PROPERTIES COMPILE_DEFINITIONS "BOOST_TEST_DYN_LINK")
target_link_libraries(Test
			clustering
			descriptor
			factories
			io
			metrics
			utils
			${PCL_LIBRARIES}
			${OpenCV_LIBS}
			${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

# Set OpenMP flags so Descriptor can be linked
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# Target Descriptor
add_executable(Descriptor ${DESCRIPTOR_SRC})
target_link_libraries(Descriptor
			clustering
			descriptor
			factories
			io
			metrics
			utils
			${PCL_LIBRARIES}
			${YAML_CPP_LIBRARIES}
			${OpenCV_LIBS})
