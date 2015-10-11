cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(PCL_DESCRIPTOR)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PCL_BUILD_TYPE Release)

file(GLOB PCL_DESCRIPTOR_SRC
    "*.h"
    "*.cpp"
)

add_subdirectory(clustering)
add_subdirectory(descriptor)
add_subdirectory(factories)
add_subdirectory(io)
add_subdirectory(utils)

add_executable(Descriptor ${PCL_DESCRIPTOR_SRC})
target_link_libraries(Descriptor clustering)
target_link_libraries(Descriptor descriptor)
target_link_libraries(Descriptor factories)
target_link_libraries(Descriptor io)
target_link_libraries(Descriptor utils)
target_link_libraries(Descriptor ${PCL_LIBRARIES})
target_link_libraries(Descriptor ${OpenCV_LIBS})
