file(GLOB CLUSTERING_SRC
    "*.h"
    "*.cpp"
)

SET_SOURCE_FILES_PROPERTIES(
  ${CLUSTERING_SRC}
  PROPERTIES
  COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}"
)

add_library(clustering ${CLUSTERING_SRC})
target_link_libraries(clustering
			utils
			metrics
			${OpenCV_LIBS})