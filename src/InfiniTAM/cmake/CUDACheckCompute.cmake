#############################
#Sourced from:
#https://raw.githubusercontent.com/jwetzl/CudaLBFGS/master/CheckComputeCapability.cmake
#############################
# Check for GPUs present and their compute capability
# based on http://stackoverflow.com/questions/2285185/easiest-way-to-test-for-existence-of-cuda-capable-gpu-from-cmake/2297877#2297877 (Christopher Bruns)

if(CUDA_FOUND)
    message(STATUS "${CMAKE_MODULE_PATH}/cuda_compute_capability.c")
    try_run(RUN_RESULT_VAR COMPILE_RESULT_VAR
        ${CMAKE_BINARY_DIR}
        ${CMAKE_MODULE_PATH}/cuda_compute_capability.c
        CMAKE_FLAGS
        -DINCLUDE_DIRECTORIES:STRING=${CUDA_TOOLKIT_INCLUDE}
        -DLINK_LIBRARIES:STRING=${CUDA_CUDART_LIBRARY}
        COMPILE_OUTPUT_VARIABLE COMPILE_OUTPUT_VAR
        RUN_OUTPUT_VARIABLE RUN_OUTPUT_VAR)
    message(STATUS "Compile: ${RUN_OUTPUT_VAR}")
    if (COMPILE_RESULT_VAR)
        message(STATUS "compiled -> " ${RUN_RESULT_VAR})
    else()
        message(STATUS "didn't compile")
    endif()
    # COMPILE_RESULT_VAR is TRUE when compile succeeds
    # RUN_RESULT_VAR is zero when a GPU is found
    if(COMPILE_RESULT_VAR AND NOT RUN_RESULT_VAR)
        message(STATUS "worked")
        set(CUDA_HAVE_GPU TRUE CACHE BOOL "Whether CUDA-capable GPU is present")
        set(CUDA_COMPUTE_CAPABILITY ${RUN_OUTPUT_VAR} CACHE STRING "Compute capability of CUDA-capable GPU present")
        mark_as_advanced(CUDA_COMPUTE_CAPABILITY)
    else()
        message(STATUS "didn't work")
        set(CUDA_HAVE_GPU FALSE CACHE BOOL "Whether CUDA-capable GPU is present")
    endif()
endif()
