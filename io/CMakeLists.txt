file(GLOB IO_SRC
	"src/*.cpp"
)

include_directories(include)

add_library(io ${IO_SRC})
target_link_libraries(io 
		utils
		factories
		descriptor
		${PCL_LIBRARIES}
		${OpenCV_LIBS})