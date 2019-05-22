IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

IF(WITH_OPENCL)
	include_directories(${OpenCL_INCLUDE_DIRS})
	add_definitions(-DCOMPILE_WITH_OPENCL)
ENDIF()

IF(PNG_FOUND)
  include_directories(${PNG_INCLUDE_DIRS})
  add_definitions(${PNG_DEFINITIONS})
  add_definitions(-DUSE_LIBPNG)
ENDIF()


IF(WITH_NETWORK)
	include_directories(${Boost_INCLUDE_DIR})
	link_directories(${Boost_LIBRARY_DIR})

	set(NETWORK_SRC
		UdpClient.cpp
		UdpClient.h
		)
ENDIF()


add_library(Utils
	FileUtils.cpp
	FileUtils.h
	NVTimer.h
	${NETWORK_SRC}
)

IF(WITH_NETWORK)
	target_link_libraries(Utils ${Boost_LIBRARIES})
ENDIF()

IF(PNG_FOUND)
  target_link_libraries(Utils ${PNG_LIBRARIES})
ENDIF()

IF(NOT WITH_PROFILING)
	add_definitions(-DNO_DETAILED_PROFILING)
ENDIF()

if(WITH_OPENCL_DEBUG)
	add_definitions(-DOPENCL_DEBUG)
endif()

