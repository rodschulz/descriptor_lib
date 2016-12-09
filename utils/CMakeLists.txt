file(GLOB UTILS_SRC
	"src/*.cpp"
)

include_directories(include)

add_library(utils ${UTILS_SRC})
target_link_libraries(utils
			${OPENSSL_LIBRARIES}
			${PCL_LIBRARIES}
			${YAML_CPP_LIBRARIES}
			${OpenCV_LIBS})