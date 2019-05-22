
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

set(PROJECT_NAME readData)

project(${PROJECT_NAME})

set(INCLUDE_DIR   "${CMAKE_SOURCE_DIR}/include")
set(SOURCE_DIR    "${CMAKE_SOURCE_DIR}/src")
set(EXTERNAL_DIR  "${CMAKE_SOURCE_DIR}/external")

# InfiniTAM
set(INFINITAM_BUILD_DIR   "${CMAKE_SOURCE_DIR}/../InfiniTAM/build" CACHE PATH "Enable the use of the InfiniTAM app")
set(INFINITAM_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../InfiniTAM"       CACHE PATH "Enable the use of the InfiniTAM app")

if(IS_DIRECTORY ${INFINITAM_BUILD_DIR} AND IS_DIRECTORY ${INFINITAM_INCLUDE_DIR})
	set(USE_INFINITAM ON CACHE BOOL "Enable the use of the InfiniTAM app")
else()
	set(USE_INFINITAM OFF CACHE BOOL "Enable the use of the InfiniTAM app")
endif()

set(USE_OPENCL ${USE_INFINITAM} CACHE BOOL "Enable OpenCL for InfiniTAM")
set(USE_ALTERA_OPENCL OFF CACHE BOOL "Link against the Altera OpenCL libraries")
set(USE_NETWORK OFF CACHE BOOL "Enable the network point cloud client and network engine in InfiniTAM")
set(USE_GPERFTOOLS OFF CACHE BOOL "Enable profiling with gperftools")



if(USE_INFINITAM)
	find_package(CUDA)
endif()

if(CUDA_FOUND)
	include_directories(${CUDA_INCLUDE_DIRS})
endif()


# Local includes first
include_directories( ${INCLUDE_DIR} )



# Packages
find_package(PCL 1.3 REQUIRED)
find_package(PNG REQUIRED)
find_package(OpenCV 3 REQUIRED)
if(USE_OPENCL)
	find_package(OpenCL)
	add_library(opencl UNKNOWN IMPORTED)
	set_property(TARGET opencl PROPERTY IMPORTED_LOCATION "${OpenCL_LIBRARIES}")
endif()
find_package(X11 QUIET)
find_package(OpenMP)
if(USE_INFINITAM)
	find_package(GLUT REQUIRED)
	find_package(OpenGL REQUIRED)
endif()
if(USE_NETWORK)
	find_package(Boost COMPONENTS system regex thread)
endif()

# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # Fix a bug in Ubuntu 16.04
# PNG
include_directories( ${PNG_INCLUDE_DIRS} )
add_definitions(-Dcimg_use_png ${PNG_DEFINITIONS})
# OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})
# OpenMP
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DVIENNACL_WITH_OPENMP)
    message("Using OpenMP")
endif()
# Boost
if(${Boost_FOUND})
	include_directories(${Boost_INCLUDE_DIR})
	link_directories(${Boost_LIBRARY_DIR})
else()
	set(USE_NETWORK OFF)
endif()


# Sources
if(USE_NETWORK)
	set(NETWORK_SRC
		${SOURCE_DIR}/TcpServer.cpp
		${SOURCE_DIR}/TcpClient.cpp
		${SOURCE_DIR}/PointCloudNetworkGetter.cpp
		)
endif()

set(SOURCES
	${SOURCE_DIR}/TangoData.cpp
	${SOURCE_DIR}/InfiniTAMServer.cpp
	${SOURCE_DIR}/main.cpp
	)


add_executable (${PROJECT_NAME} ${NETWORK_SRC} ${SOURCES})



# Libraries
if(USE_INFINITAM)
	target_link_libraries(${PROJECT_NAME} ${INFINITAM_BUILD_DIR}/Engine/libEngine.a)
	target_link_libraries(${PROJECT_NAME} ${INFINITAM_BUILD_DIR}/Utils/libUtils.a)
	target_link_libraries(${PROJECT_NAME} ${INFINITAM_BUILD_DIR}/ITMLib/libITMLib.a)
	target_link_libraries(${PROJECT_NAME} ${INFINITAM_BUILD_DIR}/ORUtils/libORUtils.a)
	target_link_libraries(${PROJECT_NAME} ${GLUT_LIBRARIES})
	target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES})
	include_directories(${INFINITAM_INCLUDE_DIR})
	add_definitions(-DUSE_INFINITAM)
	if(CUDA_FOUND)
		target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES})
	else()
		add_definitions(-DCOMPILE_WITHOUT_CUDA)
	endif()
endif()

target_link_libraries(${PROJECT_NAME} ${PNG_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
if(X11_FOUND)
	target_link_libraries(${PROJECT_NAME} ${X11_LIBRARIES})
endif()
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

if(USE_OPENCL)
	if (USE_ALTERA_OPENCL)
		execute_process(
			OUTPUT_VARIABLE AOCL_COMPILE_FLAGS
			COMMAND aocl compile-config
			OUTPUT_STRIP_TRAILING_WHITESPACE)
		execute_process(
			OUTPUT_VARIABLE AOCL_LINK_FLAGS
			COMMAND aocl link-config
			OUTPUT_STRIP_TRAILING_WHITESPACE)
		string(REPLACE "lterasic" "laltera" AOCL_LINK_FLAGS ${AOCL_LINK_FLAGS})
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${AOCL_COMPILE_FLAGS}")
		set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS ${AOCL_LINK_FLAGS})
		if (CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
			target_link_libraries(${PROJECT_NAME} alteracl alterahalmmd alterammdpcie elf rt stdc++)
		else()
			#add_definitions(-DOPENCL_DEBUG)
			target_link_libraries(${PROJECT_NAME} alteracl acl_emulator_kernel_rt alterahalmmd elf rt dl stdc++)
			#target_link_libraries(${PROJECT_NAME} alteracl acl_emulator_kernel_rt alterahalmmd altera_apb_14_0_mmd elf rt dl stdc++)
		endif ()
	else()
		target_link_libraries(${PROJECT_NAME} opencl)
		include_directories(${OpenCL_INCLUDE_DIR})
	endif()
	add_definitions(-DCOMPILE_WITH_OPENCL)
	if (CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
	else()
		add_definitions(-DOPENCL_DEBUG)
	endif()
endif()

if(USE_NETWORK)
	if(${Boost_FOUND})
		target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
		add_definitions(-DWITH_NETWORK)
	endif()
endif()

if(USE_GPERFTOOLS)
	target_link_libraries(${PROJECT_NAME} profiler)
	add_definitions(-DWITH_GPERFTOOLS)
endif()

#SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
#SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")




# C++ 11/14
set(CXX_VERSION "11")
include(CheckCXXCompilerFlag)
unset(COMPILER_SUPPORTS_FLAG CACHE)
CHECK_CXX_COMPILER_FLAG("-std=c++${CXX_VERSION}" COMPILER_SUPPORTS_FLAG)
if(COMPILER_SUPPORTS_FLAG)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++${CXX_VERSION}")
else()
	message(SEND_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++${CXX_VERSION} support. Please use a different C++ compiler.")
endif()


