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
outBasename      = "combined"             # Base name of generated folders
outKnobFilename  = "ITMSceneReconstructionEngine_OpenCL_knobs.h"                  # Name of generated knob file
logFilename      = "params.log"               # Log file to copy useful information


# ***************************************************************************
# Knobs
# ***********



KNOB3_PROJECTED_SDF_LOCAL    = [0, 1]
KNOB3_HARDCODE_DEPTH_SIZE    = [0, 1]
KNOB3_DEPTH_LOCAL            = [0, 1]
KNOB3_ENTRYID_TYPE           = [0]
KNOB3_XYZ_TYPE               = [0, 1]
KNOB3_VOXEL_BLOCK_MEM        = [0, 1, 2]
KNOB3_UNROLL_ENTRYID         = [1, 2]
KNOB3_XYZ_FLATTEN_LOOP       = [0, 1]
KNOB3_UNROLL_XYZ             = [1, 2]
KNOB3_UNROLL_VOXEL_BLOCK     = [1, 2]
KNOB3_UNROLL_VOXEL_BLOCK_OUT = [1, 2]



allCombinations = list(itertools.product(
    KNOB3_PROJECTED_SDF_LOCAL   , # 0
    KNOB3_HARDCODE_DEPTH_SIZE   , # 1
    KNOB3_DEPTH_LOCAL           , # 2
    KNOB3_ENTRYID_TYPE          , # 3
    KNOB3_XYZ_TYPE              , # 4
    KNOB3_VOXEL_BLOCK_MEM       , # 5
    KNOB3_UNROLL_ENTRYID        , # 6
    KNOB3_XYZ_FLATTEN_LOOP      , # 7
    KNOB3_UNROLL_XYZ            , # 8
    KNOB3_UNROLL_VOXEL_BLOCK    , # 9
    KNOB3_UNROLL_VOXEL_BLOCK_OUT  # 10
 ))

# ***************************************************************************


def removeCombinations(combs):

    finalList = []

    for c in combs:
        copyit = True

        if c[0 ] == 1 and c[1] != 1: copyit = False
        if c[2 ] == 1 and c[1] != 1: copyit = False
        if c[6 ] != 1 and c[3] != 0: copyit = False
        if c[7 ] == 1 and c[4] != 0: copyit = False
        if c[8 ] != 1 and c[4] != 0: copyit = False
        if c[9 ] != 1 and (c[4] != 0 or c[5] == 0): copyit = False
        if c[10] != 1 and (c[4] != 0 or c[5] == 0): copyit = False
        
        
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






