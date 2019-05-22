#############################
# Specify the project files #
#############################

##
SET(ORUTILS_HEADERS
Vector.h
Matrix.h
Cholesky.h
MathUtils.h
Image.h
CUDADefines.h
LexicalCast.h
MemoryBlock.h
MemoryBlockPersister.h
PlatformIndependence.h
OpenCLContext.h
)

#################################################################
# Collect the project files into common, CPU-only and CUDA-only #
#################################################################

set(ORUTILS_OBJECTS
Dummy.cpp
OpenCLContext.cpp
${ORUTILS_HEADERS}
)

set(ORUTILS_CL_FILES
	${PROJECT_SOURCE_DIR}/ITMLib/Engine/DeviceSpecific/OpenCL/ITMSceneReconstructionEngine_OpenCL.cl
	)


IF(WITH_OPENCL)

	# **********
	# This embeds the cl files directly into the code
	# Removed for now, but can be restored later.
	# HOWEVER, does not embed the header files included inside the cl files...
	# **********

	#add_executable(embedfile ${PROJECT_SOURCE_DIR}/Utils/embedfile.c)
	#foreach(cl_file ${ORUTILS_CL_FILES})
	#	file(RELATIVE_PATH src_file ${CMAKE_BINARY_DIR} ${cl_file})
	#	get_filename_component(basename ${cl_file} NAME_WE)
	#	set(out_file "${CMAKE_BINARY_DIR}/${basename}.c")
	#	add_custom_command(
	#		OUTPUT ${out_file}
	#		COMMAND embedfile ${basename} ${src_file}
	#		DEPENDS ${cl_file}
	#		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	#		)
	#	list(APPEND ORUTILS_CL_OBJECTS "${out_file}")
	#endforeach(cl_file)

ENDIF()

#set_property(SOURCE ${ORUTILS_OBJECTS} APPEND PROPERTY OBJECT_DEPENDS ${ORUTILS_CL_OBJECTS})

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${ORUTILS_HEADERS})

##############################################################
# Specify the include directories, target and link libraries #
##############################################################

add_library(ORUtils ${ORUTILS_OBJECTS} ${ORUTILS_CL_OBJECTS}) # 

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
  #  cuda_add_library(ITMLib
  #	${ITMLIB_CPU_OBJECTS}
  #	${ITMLIB_CUDA_OBJECTS}
  #	${ITMLIB_COMMON_OBJECTS}
  #	OPTIONS -gencode arch=compute_11,code=compute_11 -gencode arch=compute_30,code=compute_30)
  target_link_libraries(ORUtils ${CUDA_LIBRARY})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

IF(WITH_OPENCL)
	include_directories(${OpenCL_INCLUDE_DIRS})
	target_link_libraries(ORUtils ${OpenCL_LIBRARIES})
	add_definitions(-DCOMPILE_WITH_OPENCL)
ELSE()
ENDIF()

if(WITH_OPENCL_DEBUG)
	add_definitions(-DOPENCL_DEBUG)
endif()


#target_link_libraries(ITMLib Utils)
