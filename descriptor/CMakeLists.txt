file(GLOB DESCRIPTOR_SRC
	"src/*.cpp"
)

include_directories(include)

add_library(descriptor ${DESCRIPTOR_SRC})
target_link_libraries(descriptor
			utils
			${PCL_LIBRARIES})