#include "CudaUtils.h"

#include "../../../Utils/ITMLibDefines.h"

bool getCudaMemoryUsage(size_t& free_byte, size_t& total_byte)
{
    return (cudaMemGetInfo(&free_byte, &total_byte) == cudaSuccess);
}