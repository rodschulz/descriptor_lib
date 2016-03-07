file(GLOB UTILS_SRC
    "*.h"
    "*.cpp"
)

add_library(utils ${UTILS_SRC})
target_link_libraries(utils
			${PCL_LIBRARIES}
			${OpenCV_LIBS})