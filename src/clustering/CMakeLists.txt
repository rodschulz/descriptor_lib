file(GLOB CLUSTERING_SRC
    "*.h"
    "*.cpp"
)

add_library(clustering ${CLUSTERING_SRC})
target_link_libraries(clustering
			utils
			${OpenCV_LIBS})