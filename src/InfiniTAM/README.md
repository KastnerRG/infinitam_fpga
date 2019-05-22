
This is a copy of the [InfiniTAM v2 code base](https://github.com/victorprad/InfiniTAM/tree/infinitam_v2) with modifications to add OpenCL kernels. The OpenCL kernels are designed to be compiled on FPGA, if possible on an FPGA SoC board with shared memory.

## Compile

The CMakeFiles.txt has been modified to add the following variables:

* USE\_ALTERA\_OPENCL: Link against the Intel/Altera OpenCL libraries 
* WITH\_NETWORK: Enable a module that can send a result image through the network.
* WITH\_OPENCL: Enable OpenCL support
* WITH\_OPENCL\_DEBUG: Replace shared memory access by memory copy between device and host. Note that these copies are not weel optimized and slow down the application.


```
mkdir build

cd build
```

Recommended: use `ccmake ..` or `cmake-gui ..` to setup the variables mentioned aboved.

Otherwise, at minimum: `cmake -DCMAKE_BUILD_TYPE=Release ..`

