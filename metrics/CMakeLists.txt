file(GLOB METRICS_SRC
	"src/*.cpp"
)

include_directories(include)

add_library(metrics ${METRICS_SRC})
target_link_libraries(metrics
			${OpenCV_LIBS})