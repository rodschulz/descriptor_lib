file(GLOB FACTORIES_SRC
    "*.h"
    "*.cpp"
)

add_library(factories ${FACTORIES_SRC})
target_link_libraries(factories
			utils
			${PCL_LIBRARIES})