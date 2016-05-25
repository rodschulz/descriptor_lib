file(GLOB UTILS_SRC
	"src/*.cpp"
)

file(GLOB UTILS_INC
	"include/*.h"
	"include/*.hpp"
)

include_directories(include)

add_library(utils ${UTILS_SRC})
target_link_libraries(utils
			${PCL_LIBRARIES}
			${YAML_CPP_LIBRARIES}
			${OpenCV_LIBS})