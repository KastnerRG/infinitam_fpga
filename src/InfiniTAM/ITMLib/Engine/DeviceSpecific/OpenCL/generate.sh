#!/bin/bash

g++ -x c++ -E -C -DALTERA_CL ITMSceneReconstructionEngine_OpenCL.cl | grep -v '^# [0-9]' > output.cl

