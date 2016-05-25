file(GLOB METRICS_SRC
	"src/*.cpp"
)

file(GLOB METRICS_INC
	"include/*.h"
	"include/*.hpp"
)

include_directories(include)

add_library(metrics ${METRICS_SRC})
target_link_libraries(metrics
			utils
			${OpenCV_LIBS})