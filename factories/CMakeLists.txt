file(GLOB FACTORIES_SRC
	"src/*.cpp"
)

include_directories(include)

add_library(factories ${FACTORIES_SRC})
target_link_libraries(factories
			utils
			descriptor
			${PCL_LIBRARIES})