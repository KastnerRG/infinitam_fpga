
This directory contains the code for an program that reads data from files or the network and send these data to InfiniTAM. It also contains a number of scripts that were used to run experiments and collect performance data from multiple kernels.

## Requirements

* PCL
* ImageMagick
* libpng
* OpenCV
* C++11

## Compile

The CMakeFiles.txt contains several variables, including:

* USE\_ALTERA\_OPENCL: Link against Intel/Altera OpenCL libraries.
* USE\_GPERFTOOLS: Enable some profiling tools.
* USE\_INFINITAM: Make sure this is on.
* USE\_NETWORK: Enable code to send/receive data through the network.
* USE\_OPENCL: Enable OpenCL (should be on if OpenCL is enabled in InfiniTAM)


Make sure you compile InfiniTAM first. Then:

```
mkdir build
cd build
```
`ccmake ..`

or

`cmake-gui ..`

or

`cmake -DCMAKE_BUILD_TYPE=Release ..`

## Example Usage

Make sure to copy the [infinitam\_params.txt](src/infinitam_params.txt) file into your working directory. You can modify several options in this file, including the device on which InfiniTAM runs (CPU/GPU/FPGA).

### TUM benchmarks

Assuming you have downloaded one of the TUM benchmarks in the `~/data` directory:


```
./readData -i ~/data/rgbd_dataset_freiburg1_room/depth/ --input-depth-factor 0.0002 --depth-intr 640,480,517.3,516.5,318.6,255.3 --infinitam --infinitam-params infinitam_params.txt --scale 0.5 --max-frames 700
```

### Other

[Google Tango Sample Data (Google Drive link)](https://drive.google.com/file/d/1T1tdXZcax1XoaN2SoEIspaXYkn3eHt_i/view?usp=sharing)

Assuming you have downloaded the link above in the `~/data` directory:

```
./readData -i ~/data/cave_9 --infinitam --infinitam-params infinitam_params.txt --max-frames 400
```

