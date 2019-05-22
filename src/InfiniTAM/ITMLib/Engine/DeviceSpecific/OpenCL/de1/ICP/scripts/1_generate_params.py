#!/usr/bin/python



import itertools
import sys

sys.path.append("/home/qkgautier/workspace/spector/common/scripts")
from generateDesigns import createFolders


##############
# README
##############
# See ../../common/scripts/generateDesigns.py 
##############


templateFilepath = "../src/ITMSceneReconstructionEngine_OpenCL_knobs.h.template"  # Knobs template file
kernelFilename   = "../src/ITMSceneReconstructionEngine_OpenCL.cl"      # Kernel file to copy 
dirToCopy        = "../benchmarks/basefolder" # Directory containing the source code

outRootPath      = "../benchmarks"            # Name of output directory where all folders are generated
outBasename      = "icp"             # Base name of generated folders
outKnobFilename  = "ITMSceneReconstructionEngine_OpenCL_knobs.h"                  # Name of generated knob file
logFilename      = "params.log"               # Log file to copy useful information


# ***************************************************************************
# Knobs
# ***********



KNOB4_INTERPOLATE_POINT      = [0, 1]
KNOB4_INTERPOLATE_NORMAL     = [0, 1]
KNOB4_UNROLL_NABLA           = [0, 1]
KNOB4_UNROLL_HESSIAN         = [0, 1]
KNOB4_SHORT_ITERATION_BRANCH = [0, 1]
KNOB4_SUM_NABLA_TYPE         = [0, 1, 2]
KNOB4_SUM_HESSIAN_TYPE       = [0, 1, 2]
KNOB4_SHIFT_REG_SIZE         = [2, 4, 8, 16]
KNOB4_FLATTEN_LOOP           = [0, 1]
KNOB4_USE_ND_RANGE           = [0] # [0, 1] -------- DISABLING ND RANGE FOR NOW!
KNOB4_PIPE_DEPTH             = [2, 32, 128, 512]
KNOB4_COMPUTE_UNITS          = [1]





allCombinations = list(itertools.product(
 KNOB4_INTERPOLATE_POINT     , # 0
 KNOB4_INTERPOLATE_NORMAL    , # 1
 KNOB4_UNROLL_NABLA          , # 2
 KNOB4_UNROLL_HESSIAN        , # 3
 KNOB4_SHORT_ITERATION_BRANCH, # 4
 KNOB4_SUM_NABLA_TYPE        , # 5
 KNOB4_SUM_HESSIAN_TYPE      , # 6
 KNOB4_SHIFT_REG_SIZE        , # 7
 KNOB4_FLATTEN_LOOP          , # 8
 KNOB4_USE_ND_RANGE          , # 9
 KNOB4_PIPE_DEPTH            , # 10
 KNOB4_COMPUTE_UNITS           # 11
 ))

# ***************************************************************************


def removeCombinations(combs):

    finalList = []

    for c in combs:
        copyit = True

        if c[5 ] != 2 and c[6] != 2 and c[7] != 2: copyit = False
        if c[9 ] == 1 and c[8] != 0: copyit = False
        if c[9 ] == 0 and c[10] != 2: copyit = False
        
        if copyit:
            finalList.append(c)

    return finalList




def main():

    doCreateFolders = 0

    if len(sys.argv) > 1:
        doCreateFolders = int(sys.argv[1])

    finalCombinations = removeCombinations(allCombinations)

    print("Num combinations: " + str(len(finalCombinations)))
    print("vs " + str(len(allCombinations)))


    if doCreateFolders == 1:
        createFolders(
                finalCombinations,
                templateFilepath,
                kernelFilename,
                dirToCopy,
                outRootPath,
                outBasename,
                outKnobFilename,
                logFilename)
    else:
        print("\nNote: To actually create the folders, run:\n" + sys.argv[0] + " 1\n")





if __name__ == "__main__":
    main()






