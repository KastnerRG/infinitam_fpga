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
outBasename      = "raycasting"             # Base name of generated folders
outKnobFilename  = "ITMSceneReconstructionEngine_OpenCL_knobs.h"                  # Name of generated knob file
logFilename      = "params.log"               # Log file to copy useful information


# ***************************************************************************
# Knobs
# ***********

KNOB2_INTERPOLATE       = [0, 1]
KNOB2_RAYCAST_REFINE    = [0, 1]
KNOB2_USE_INDEX_CACHE   = [0, 1]
KNOB2_CACHE_HASH_ENTRY  = [0, 1]
KNOB2_CACHE_VOXEL_DATA  = [0, 1]
KNOB2_HARDCODE_IMG_SIZE = [0, 1]
KNOB2_USE_ND_RANGE      = [0, 1]
KNOB2_USE_MINMAXIMG     = [0, 1, 2]




allCombinations = list(itertools.product(
 KNOB2_INTERPOLATE      , # 0
 KNOB2_RAYCAST_REFINE   , # 1
 KNOB2_USE_INDEX_CACHE  , # 2
 KNOB2_CACHE_HASH_ENTRY , # 3
 KNOB2_CACHE_VOXEL_DATA , # 4
 KNOB2_HARDCODE_IMG_SIZE, # 5
 KNOB2_USE_ND_RANGE     , # 6
 KNOB2_USE_MINMAXIMG      # 7
 ))

# ***************************************************************************


def removeCombinations(combs):

    finalList = []

    for c in combs:
        copyit = True

        if False: copyit = False
        
        
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






