# Required version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Colors defined for message
if(NOT WIN32)
	string(ASCII 27 Esc)
	set(ColorReset	"${Esc}[m")
	set(Red 		"${Esc}[31m")
	set(Green 		"${Esc}[32m")
	set(Yellow      "${Esc}[33m")
	set(Blue 		"${Esc}[34m")
	set(Magenta 	"${Esc}[35m")
	set(Cyan 		"${Esc}[36m")
endif()

# Add additional path for cmake to find OpenCV
set(CMAKE_PREFIX_PATH "/opt/ros/hydro/" ${CMAKE_PREFIX_PATH})

# Extra definition to show all the warnings
add_definitions("-Wall")

# Find PCL if not already found
if (NOT PCL_FOUND)
	message(STATUS "${Yellow}PCL not found yet, searching for package${ColorReset}")
	find_package(PCL 1.7 REQUIRED)

	if (PCL_FOUND)
		message(STATUS "${Cyan}\tPCL found version ${PCL_VERSION}${ColorReset}")
		message(STATUS "${Cyan}\tPCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}${ColorReset}")

		# Set the desired build type to use for PCL
		set(PCL_BUILD_TYPE Release)

		# Add extra required definitions
		add_definitions(${PCL_DEFINITIONS})

		# Add include and linking directories
		include_directories(${PCL_INCLUDE_DIRS})
		link_directories(${PCL_LIBRARY_DIRS})
	endif()
endif()

# Find OpenCV if not already found
if (NOT OpenCV_FOUND)
	message(STATUS "${Yellow}OpenCV not found yet, searching for package${ColorReset}")
	find_package(OpenCV 2.4.8 REQUIRED)

	if (OpenCV_FOUND)
		message(STATUS "${Cyan}\tFound OpenCV version ${OpenCV_VERSION}${ColorReset}")
		message(STATUS "${Cyan}\tOpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}${ColorReset}")

		# Add include directory
		include_directories(${OpenCV_INCLUDE_DIRS})
	endif()
endif()

# Find YamlCpp if not already found
if (NOT yaml-cpp_FOUND)
	message(STATUS "${Yellow}YamlCpp not found yet, searching for package${ColorReset}")
	find_package(yaml-cpp 0.5.2 REQUIRED)
	
	if (yaml-cpp_FOUND)
		message(STATUS "${Cyan}\tFound yaml-cpp version ${yaml-cpp_VERSION}${ColorReset}")
		message(STATUS "${Cyan}\tYAML_CPP_INCLUDE_DIR = ${YAML_CPP_INCLUDE_DIR}${ColorReset}")

		# Add include directory
		include_directories(${YAML_CPP_INCLUDE_DIR})
	endif()
endif()

if (NOT Boost_FOUND)
	message(STATUS "${Yellow}Boost not found yet, searching for package${ColorReset}")
	find_package(Boost 1.46 COMPONENTS unit_test_framework REQUIRED)

	if (Boost_FOUND)
		message(STATUS "${Cyan}\tFound Boost version ${Boost_LIB_VERSION}${ColorReset}")
		message(STATUS "${Cyan}\tBoost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}${ColorReset}")
		
		# Add include and linking directories
		include_directories(${Boost_INCLUDE_DIRS})
		link_directories(${Boost_LIBRARY_DIRS})
	endif()
endif()

if (NOT OPENSSL_FOUND)
	message(STATUS "${Yellow}OpenSSL not found yet, searching for package${ColorReset}")
	find_package(OpenSSL REQUIRED)

	if (OPENSSL_FOUND)
		message(STATUS "${Cyan}\tFound OpenSSL version ${OPENSSL_VERSION}${ColorReset}")
		message(STATUS "${Cyan}\tOPENSSL_INCLUDE_DIR = ${OPENSSL_INCLUDE_DIR}${ColorReset}")

		# Add include and linking directories
		include_directories(${OPENSSL_INCLUDE_DIR})
		link_directories(${OPENSSL_LIBRARIES})
	endif()
endif()

if (NOT OPENMP_FOUND)
	message(STATUS "${Yellow}OpenMP not found yet, searching for package${ColorReset}")
	find_package(OpenMP REQUIRED)

	if (OPENMP_FOUND)
		message(STATUS "${Cyan}\tFound OpenMP${ColorReset}")

		if (${useOpenMP})
			message(STATUS "${Cyan}\t...flag 'useOpenMP' found, OpenMP enabled${ColorReset}")
			
			# Set OpenMP flags to be used
			set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
			set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		else()
			message(STATUS "${Cyan}\t...flag 'useOpenMP' not found, OpenMP disabled${ColorReset}")
		endif()
	endif()
endif()

# Get libraries to include
file(GLOB lib_directories *)
list(REMOVE_ITEM lib_directories
	${CMAKE_CURRENT_SOURCE_DIR}/.git
	${CMAKE_CURRENT_SOURCE_DIR}/test)

# Add include directories from the libraries
foreach(subdir ${lib_directories})
	if (IS_DIRECTORY ${subdir})
		include_directories(${subdir}/include)
	endif()
endforeach()

# Add subdirectories with libraries
foreach(subdir ${lib_directories})
	if (IS_DIRECTORY ${subdir})
		get_filename_component(libname ${subdir} NAME)
		message(STATUS "${Blue}Adding lib ${libname}${ColorReset}")
		add_subdirectory(${subdir})
	endif()
endforeach()
