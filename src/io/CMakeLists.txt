file(GLOB IO_SRC
    "*.h"
    "*.cpp"
)

add_library(io ${IO_SRC})
target_link_libraries(io utils ${PCL_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(io factories)
target_link_libraries(io descriptor)