file(GLOB DESCRIPTOR_SRC
    "*.h"
    "*.cpp"
)

add_library(descriptor ${DESCRIPTOR_SRC})