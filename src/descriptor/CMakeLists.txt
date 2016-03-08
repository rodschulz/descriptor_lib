file(GLOB DESCRIPTOR_SRC
    "*.h"
    "*.cpp"
)

add_library(descriptor ${DESCRIPTOR_SRC})
target_link_libraries(descriptor
			utils
			${PCL_LIBRARIES})