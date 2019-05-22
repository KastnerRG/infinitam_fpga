cmake_minimum_required(VERSION 2.8)
project(InfiniTAM)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

macro(CANONIFY_BOOL var)
  if (${var})
    set(${var} TRUE)
  else()
    set(${var} FALSE)
  endif()
endmacro()

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
	set(DEFAULT_OPENCL_DEBUG off)
else()
	set(DEFAULT_OPENCL_DEBUG on)
endif()


find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

find_package(CUDA QUIET)
CANONIFY_BOOL(CUDA_FOUND)
message(STATUS "CUDA found: " ${CUDA_FOUND})

find_package(OpenNI QUIET)
CANONIFY_BOOL(OPENNI_FOUND)
message(STATUS "OpenNI found: " ${OPENNI_FOUND})

find_package(libuvc QUIET)
CANONIFY_BOOL(libuvc_FOUND)
message(STATUS "libuvc found: ${libuvc_FOUND}")

find_package(RealSense QUIET)
CANONIFY_BOOL(REALSENSE_FOUND)
message(STATUS "librealsense found: ${REALSENSE_FOUND}")

find_package(PNG QUIET)
CANONIFY_BOOL(PNG_FOUND)
message(STATUS "libpng found: ${PNG_FOUND}")

find_package(OpenCL QUIET)
CANONIFY_BOOL(OpenCL_FOUND)
message(STATUS "OpenCL found: ${OpenCL_FOUND}")

find_package(Boost COMPONENTS system regex thread QUIET)
CANONIFY_BOOL(Boost_FOUND)
message(STATUS "Boost found: ${Boost_FOUND}")

OPTION(WITH_CUDA   "Build with CUDA support?"   ${CUDA_FOUND})
OPTION(WITH_OPENCL "Build with OpenCL support?" ${OpenCL_FOUND})
OPTION(USE_ALTERA_OPENCL "Link against the Altera OpenCL libraries" OFF)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
OPTION(WITH_REALSENSE "Build with Intel RealSense support?" ${REALSENSE_FOUND})
OPTION(WITH_NETWORK "Include networking engine (requires boost)?" ${Boost_FOUND})
OPTION(WITH_OPENCL_DEBUG "Include memory transfers to debug OpenCL code on PCIe devices?" ${DEFAULT_OPENCL_DEBUG})
OPTION(WITH_PROFILING "With profiling counters?" ON)

IF(MSVC_IDE)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-DUSING_CMAKE=1)
ELSE(MSVC_IDE)
  set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  set(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #set(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
  #SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
ENDIF(MSVC_IDE)

if(APPLE)
  set(CUDA_HOST_COMPILER /usr/bin/clang)
endif(APPLE)

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
ENDIF()

if(USE_ALTERA_OPENCL)
	execute_process(
		OUTPUT_VARIABLE AOCL_COMPILE_FLAGS
		COMMAND aocl compile-config
		OUTPUT_STRIP_TRAILING_WHITESPACE)
	execute_process(
		OUTPUT_VARIABLE AOCL_LINK_FLAGS
		COMMAND aocl linkflags
		OUTPUT_STRIP_TRAILING_WHITESPACE)
	#set_target_properties(ITMLib PROPERTIES compile_flags ${AOCL_COMPILE_FLAGS})
	add_compile_options(${AOCL_COMPILE_FLAGS} -Wno-unknown-pragmas)
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${AOCL_LINK_FLAGS}")
endif()

add_subdirectory(ORUtils)
add_subdirectory(ITMLib)
add_subdirectory(Utils)
add_subdirectory(Engine)

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()


IF(WITH_OPENCL)
	add_definitions(-DCOMPILE_WITH_OPENCL)
	include_directories(${OpenCL_INCLUDE_DIRS})
ELSE()
ENDIF()

if(WITH_OPENCL_DEBUG)
	add_definitions(-DOPENCL_DEBUG)
endif()



add_executable(InfiniTAM_cli InfiniTAM_cli.cpp)
target_link_libraries(InfiniTAM_cli Engine)
target_link_libraries(InfiniTAM_cli Utils)
add_executable(InfiniTAM InfiniTAM.cpp)
target_link_libraries(InfiniTAM Engine)
target_link_libraries(InfiniTAM Utils)


