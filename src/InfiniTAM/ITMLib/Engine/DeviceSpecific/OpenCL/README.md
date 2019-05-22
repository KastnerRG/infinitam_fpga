# Kernels

- The kernels are implemented in [ITMSceneReconstructionEngine\_OpenCL.cl](ITMSceneReconstructionEngine_OpenCL.cl)

- The knob values can be changed in [ITMSceneReconstructionEngine\_OpenCL\_knobs.h](ITMSceneReconstructionEngine_OpenCL_knobs.h)

- Because the kernels can be hard to read with the knobs, we provide a script *generate.sh* to pre-process the knob values and generate a design easier to read. The following command will produce an *output.cl* file on Linux:

``` ./generate.sh ```

- *example.cl* is a depth fusion kernel where the knobs have been pre-processed.

