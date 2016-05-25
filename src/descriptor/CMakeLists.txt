file(GLOB DESCRIPTOR_SRC
	"src/*.cpp"
)

file(GLOB DESCRIPTOR_INC
	"include/*.h"
	"include/*.hpp"
)

include_directories(include)

add_library(descriptor ${DESCRIPTOR_SRC})
target_link_libraries(descriptor
			utils
			${PCL_LIBRARIES})