file(GLOB METRICS_SRC
    "*.h"
    "*.cpp"
)

add_library(metrics ${METRICS_SRC})
target_link_libraries(metrics
			utils
			${OpenCV_LIBS})