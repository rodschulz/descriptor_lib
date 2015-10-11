file(GLOB FACTORIES_SRC
    "*.h"
    "*.cpp"
)

add_library(factories ${FACTORIES_SRC})