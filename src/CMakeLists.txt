cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(PCL_DESCRIPTOR)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PCL_BUILD_TYPE Release)

file(GLOB PCL_DESCRIPTOR_SRC
    "*.h"
    "*.cpp"
)
add_executable(Descriptor ${PCL_DESCRIPTOR_SRC})
target_link_libraries(Descriptor ${PCL_LIBRARIES})
