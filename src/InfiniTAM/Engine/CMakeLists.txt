include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

IF(WITH_OPENCL)
	add_definitions(-DCOMPILE_WITH_OPENCL)
	include_directories(${OpenCL_INCLUDE_DIRS})
ENDIF()

IF(OPENNI_FOUND)
  include_directories(${OpenNI_INCLUDE_DIR})
ELSE(OPENNI_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_OpenNI)
ENDIF(OPENNI_FOUND)

IF(MSKINECTAPI_FOUND)
  include_directories(${MSKINECTAPI_INCLUDE_DIR})
ELSE(MSKINECTAPI_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_Kinect2)
ENDIF(MSKINECTAPI_FOUND)

IF(DEFINED libuvc_INCLUDE_DIRS)
  include_directories(${libuvc_INCLUDE_DIRS})
  add_definitions(-DCOMPILE_WITH_LibUVC)
ENDIF()

IF(WITH_REALSENSE)
  include_directories(${RealSense_INCLUDE_DIR})
  add_definitions(-DCOMPILE_WITH_RealSense)
  SET_PROPERTY(SOURCE RealSenseEngine.cpp PROPERTY COMPILE_FLAGS -std=c++11)
ENDIF()

IF(WITH_NETWORK)
	include_directories(${Boost_INCLUDE_DIR})
	link_directories(${Boost_LIBRARY_DIR})

	set(NETWORK_SRC
		NetworkEngine.cpp
		NetworkEngine.h
		)
ENDIF()


add_library(Engine
ImageSourceEngine.cpp
ImageSourceEngine.h
IMUSourceEngine.cpp
IMUSourceEngine.h
Kinect2Engine.cpp
Kinect2Engine.h
OpenNIEngine.cpp
OpenNIEngine.h
LibUVCEngine.cpp
LibUVCEngine.h
UIEngine.cpp
UIEngine.h
CLIEngine.cpp
CLIEngine.h
RealSenseEngine.cpp
RealSenseEngine.h
${NETWORK_SRC}
)

target_link_libraries(Engine ${GLUT_LIBRARIES})
target_link_libraries(Engine ${OPENGL_LIBRARIES})
target_link_libraries(Engine ITMLib)
IF(WITH_CUDA)
  target_link_libraries(Engine ${CUDA_LIBRARY})
ENDIF()
IF(OPENNI_FOUND)
  target_link_libraries(Engine ${OpenNI_LIBRARY})
ENDIF(OPENNI_FOUND)
IF(DEFINED libuvc_LIBRARIES)
  target_link_libraries(Engine ${libuvc_LIBRARIES})
ENDIF()
IF(WITH_REALSENSE)
  TARGET_LINK_LIBRARIES(Engine ${RealSense_LIBRARY})
ENDIF()
IF(WITH_NETWORK)
	target_link_libraries(Engine ${Boost_LIBRARIES})
ENDIF()

if(WITH_OPENCL_DEBUG)
	add_definitions(-DOPENCL_DEBUG)
endif()

