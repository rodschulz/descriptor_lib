file(GLOB FACTORIES_SRC
	"src/*.cpp"
)

file(GLOB FACTORIES_INC
	"include/*.h"
	"include/*.hpp"
)

include_directories(include)

add_library(factories ${FACTORIES_SRC})
target_link_libraries(factories
			utils
			${PCL_LIBRARIES})