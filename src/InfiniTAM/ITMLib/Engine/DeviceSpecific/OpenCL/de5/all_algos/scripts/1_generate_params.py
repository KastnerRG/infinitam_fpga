#!/usr/bin/python3



import itertools
import sys
#from constraint import Problem


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
outBasename      = "all_algos"             # Base name of generated folders
outKnobFilename  = "ITMSceneReconstructionEngine_OpenCL_knobs.h"                  # Name of generated knob file
logFilename      = "params.log"               # Log file to copy useful information


# ***************************************************************************
# Knobs
# ***********

KNOB_SIMD                   = [1]
KNOB_COMPUTE_UNITS          = [1, 2]
KNOB_XYZ_TYPE               = [0, 1]
KNOB_ENTRYID_TYPE           = [0, 1, 2]
KNOB_ENTRYID_NUM_WORK_ITEMS = [64, 128]
KNOB_HARDCODE_DEPTH_SIZE    = [0, 1]
KNOB_VOXEL_BLOCK_MEM        = [0, 1, 2]
KNOB_XYZ_FLATTEN_LOOP       = [0, 1]
KNOB_UNROLL_XYZ             = [1, 2]
KNOB_UNROLL_ENTRYID         = [1, 2]
KNOB_UNROLL_VOXEL_BLOCK     = [1, 2]
KNOB_UNROLL_VOXEL_BLOCK_OUT = [1, 2]
KNOB_DEPTH_LOCAL            = [0, 1]
KNOB_MAX_W_CHECK_POSITION   = [0, 1]

KNOB2_INTERPOLATE       = [0, 1]
KNOB2_RAYCAST_REFINE    = [0, 1]
KNOB2_USE_INDEX_CACHE   = [0, 1]
KNOB2_CACHE_HASH_ENTRY  = [0, 1]
KNOB2_CACHE_VOXEL_DATA  = [0, 1]
KNOB2_HARDCODE_IMG_SIZE = [0, 1]
KNOB2_USE_ND_RANGE      = [0, 1]
KNOB2_USE_MINMAXIMG     = [0, 1, 2]

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
KNOB4_PIPE_DEPTH             = [2] #, 32, 128, 512]
KNOB4_COMPUTE_UNITS          = [1]


KNOB_COMBS = [
        [1,1,0,64,1,0,1,1,1,1,1,0,0],
        [1,1,0,64,1,0,1,1,2,1,1,0,0],
        [1,1,0,64,1,1,1,1,1,1,1,0,1],
        [1,1,0,64,1,1,1,1,2,1,1,0,1],
        [1,1,2,64,0,0,1,1,1,1,1,0,0],
        [1,1,2,64,0,1,1,1,1,1,1,0,1],
        [1,1,2,64,1,0,1,1,1,1,1,0,0],
        [2,1,2,64,0,2,1,1,1,1,1,0,1],
        [2,1,2,64,1,0,1,1,1,1,1,0,0],
        [2,1,2,64,1,2,1,1,1,1,1,0,0]]

KNOB2_COMBS = [
        [0,0,1,1,1,1,1,1],
        [0,0,0,0,0,0,0,1],
        [0,0,0,1,0,1,1,2],
        [0,0,1,1,1,0,1,2],
        [0,0,0,1,0,0,1,1],
        [0,0,0,1,1,1,1,2],
        [0,0,1,0,1,0,1,1],
        [0,0,0,0,1,1,1,2],
        [0,0,0,0,0,1,1,1],
        [0,0,0,1,0,1,1,1],
        [0,0,0,1,1,0,1,2],
        [0,0,0,0,1,1,0,1],
        [0,0,1,0,1,0,0,2],
        [0,0,0,1,1,1,1,1],
        [0,0,1,0,1,0,1,0]
        ]

KNOB4_COMBS = [
        [0,0,1,0,0,2,0,2,1,0,2,1],
        [0,0,1,1,0,1,0,2,1,0,2,1],
        [0,0,0,1,1,2,1,4,1,0,2,1],
        [0,0,1,1,1,1,1,2,1,0,2,1],
        [0,1,0,1,0,0,2,2,1,0,2,1],
        [0,0,1,1,0,2,2,8,1,0,2,1],
        [0,1,0,1,1,1,1,2,0,0,2,1],
        [0,1,1,1,1,2,2,2,1,0,2,1],
        [0,0,1,1,1,2,2,4,1,0,2,1],
        [0,1,1,1,0,0,2,2,1,0,2,1],
        [0,0,1,0,1,1,0,2,0,0,2,1],
        [0,0,1,1,0,0,0,2,0,0,2,1],
        [0,1,0,1,0,1,2,8,1,0,2,1],
        [0,1,1,1,0,2,2,4,1,0,2,1],
        [1,1,1,1,0,2,2,2,1,0,2,1],
        [0,1,1,1,0,2,2,2,0,0,2,1],
        [0,1,1,1,0,1,1,2,1,0,2,1],
        [0,0,0,1,1,1,1,2,1,0,2,1],
        [0,0,1,1,0,0,1,2,0,0,2,1],
        [0,1,1,0,0,0,0,2,0,0,2,1]
        ]


#allCombinations = itertools.product(
# KNOB_SIMD                     , # 0 
# KNOB_COMPUTE_UNITS            , # 1  
# KNOB_XYZ_TYPE                 , # 2 
# KNOB_ENTRYID_TYPE             , # 3
# KNOB_ENTRYID_NUM_WORK_ITEMS   , # 4 
# KNOB_HARDCODE_DEPTH_SIZE      , # 5
# KNOB_VOXEL_BLOCK_MEM          , # 6 
# KNOB_XYZ_FLATTEN_LOOP         , # 7
# KNOB_UNROLL_XYZ               , # 8
# KNOB_UNROLL_ENTRYID           , # 9
# KNOB_UNROLL_VOXEL_BLOCK       , # 10
# KNOB_UNROLL_VOXEL_BLOCK_OUT   , # 11
# KNOB_DEPTH_LOCAL              , # 12
# KNOB_MAX_W_CHECK_POSITION     , # 13
#
# KNOB2_INTERPOLATE      , # 14
# KNOB2_RAYCAST_REFINE   , # 15
# KNOB2_USE_INDEX_CACHE  , # 16
# KNOB2_CACHE_HASH_ENTRY , # 17
# KNOB2_CACHE_VOXEL_DATA , # 18
# KNOB2_HARDCODE_IMG_SIZE, # 19
# KNOB2_USE_ND_RANGE     , # 20
# KNOB2_USE_MINMAXIMG    , # 21
#
# KNOB4_INTERPOLATE_POINT     , # 22
# KNOB4_INTERPOLATE_NORMAL    , # 23
# KNOB4_UNROLL_NABLA          , # 24
# KNOB4_UNROLL_HESSIAN        , # 25
# KNOB4_SHORT_ITERATION_BRANCH, # 26
# KNOB4_SUM_NABLA_TYPE        , # 27
# KNOB4_SUM_HESSIAN_TYPE      , # 28
# KNOB4_SHIFT_REG_SIZE        , # 29
# KNOB4_FLATTEN_LOOP          , # 30
# KNOB4_USE_ND_RANGE          , # 31
# KNOB4_PIPE_DEPTH            , # 32
# KNOB4_COMPUTE_UNITS           # 33
# )

# ***************************************************************************


def removeCombinations(combs):

    finalList = []

    for c in combs:
        copyit = True

        # Depth Fusion conditions
        if c[2] == 1 and c[3] == 1: copyit = False
        if c[0] > 1 and c[2] == 0 and c[3] == 0: copyit = False
        if c[1] > 1 and c[3] < 2: copyit = False
        if c[3] != 1 and c[4] != KNOB_ENTRYID_NUM_WORK_ITEMS[0]: copyit = False
        if c[5] == 0 and c[12] != 0: copyit = False
        if c[7] != 1 and c[2] != 0: copyit = False
        if c[8] > 1 and c[2] != 0: copyit = False
        if c[9] > 1 and c[3] != 0: copyit = False
        if c[10] > 1 and (c[2] != 0 or c[6] == 0): copyit = False
        if c[11] > 1 and (c[2] != 0 or c[6] == 0): copyit = False


        # ICP conditions
        if c[27 ] != 2 and c[28] != 2 and c[29] != 2: copyit = False
        if c[31 ] == 1 and c[32] != 0: copyit = False
        if c[31 ] == 0 and c[32] != 2: copyit = False
        
        if copyit:
            finalList.append(c)

    return finalList




def main():

    doCreateFolders = 0

    if len(sys.argv) > 1:
        doCreateFolders = int(sys.argv[1])




    #prob = Problem()


    #prob.addVariable("KNOB_SIMD"                   ,  KNOB_SIMD                  )
    #prob.addVariable("KNOB_COMPUTE_UNITS"          ,  KNOB_COMPUTE_UNITS         )
    #prob.addVariable("KNOB_XYZ_TYPE"               ,  KNOB_XYZ_TYPE              )
    #prob.addVariable("KNOB_ENTRYID_TYPE"           ,  KNOB_ENTRYID_TYPE          )
    #prob.addVariable("KNOB_ENTRYID_NUM_WORK_ITEMS" ,  KNOB_ENTRYID_NUM_WORK_ITEMS)
    #prob.addVariable("KNOB_HARDCODE_DEPTH_SIZE"    ,  KNOB_HARDCODE_DEPTH_SIZE   )
    #prob.addVariable("KNOB_VOXEL_BLOCK_MEM"        ,  KNOB_VOXEL_BLOCK_MEM       )
    #prob.addVariable("KNOB_XYZ_FLATTEN_LOOP"       ,  KNOB_XYZ_FLATTEN_LOOP      )
    #prob.addVariable("KNOB_UNROLL_XYZ"             ,  KNOB_UNROLL_XYZ            )
    #prob.addVariable("KNOB_UNROLL_ENTRYID"         ,  KNOB_UNROLL_ENTRYID        )
    #prob.addVariable("KNOB_UNROLL_VOXEL_BLOCK"     ,  KNOB_UNROLL_VOXEL_BLOCK    )
    #prob.addVariable("KNOB_UNROLL_VOXEL_BLOCK_OUT" ,  KNOB_UNROLL_VOXEL_BLOCK_OUT)
    #prob.addVariable("KNOB_DEPTH_LOCAL"            ,  KNOB_DEPTH_LOCAL           )
    #prob.addVariable("KNOB_MAX_W_CHECK_POSITION"   ,  KNOB_MAX_W_CHECK_POSITION  )

    #prob.addVariable("KNOB2_INTERPOLATE"           ,  KNOB2_INTERPOLATE      )
    #prob.addVariable("KNOB2_RAYCAST_REFINE"        ,  KNOB2_RAYCAST_REFINE   )
    #prob.addVariable("KNOB2_USE_INDEX_CACHE"       ,  KNOB2_USE_INDEX_CACHE  )
    #prob.addVariable("KNOB2_CACHE_HASH_ENTRY"      ,  KNOB2_CACHE_HASH_ENTRY )
    #prob.addVariable("KNOB2_CACHE_VOXEL_DATA"      ,  KNOB2_CACHE_VOXEL_DATA )
    #prob.addVariable("KNOB2_HARDCODE_IMG_SIZE"     ,  KNOB2_HARDCODE_IMG_SIZE)
    #prob.addVariable("KNOB2_USE_ND_RANGE"          ,  KNOB2_USE_ND_RANGE     )
    #prob.addVariable("KNOB2_USE_MINMAXIMG"         ,  KNOB2_USE_MINMAXIMG    )

    #prob.addVariable("KNOB4_INTERPOLATE_POINT"     ,  KNOB4_INTERPOLATE_POINT      )    
    #prob.addVariable("KNOB4_INTERPOLATE_NORMAL"    ,  KNOB4_INTERPOLATE_NORMAL     )   
    #prob.addVariable("KNOB4_UNROLL_NABLA"          ,  KNOB4_UNROLL_NABLA           )  
    #prob.addVariable("KNOB4_UNROLL_HESSIAN"        ,  KNOB4_UNROLL_HESSIAN         ) 
    #prob.addVariable("KNOB4_SHORT_ITERATION_BRANCH",  KNOB4_SHORT_ITERATION_BRANCH ) 
    #prob.addVariable("KNOB4_SUM_NABLA_TYPE"        ,  KNOB4_SUM_NABLA_TYPE         )
    #prob.addVariable("KNOB4_SUM_HESSIAN_TYPE"      ,  KNOB4_SUM_HESSIAN_TYPE       )
    #prob.addVariable("KNOB4_SHIFT_REG_SIZE"        ,  KNOB4_SHIFT_REG_SIZE         )
    #prob.addVariable("KNOB4_FLATTEN_LOOP"          ,  KNOB4_FLATTEN_LOOP           )
    #prob.addVariable("KNOB4_USE_ND_RANGE"          ,  KNOB4_USE_ND_RANGE           )
    #prob.addVariable("KNOB4_PIPE_DEPTH"            ,  KNOB4_PIPE_DEPTH             )
    #prob.addVariable("KNOB4_COMPUTE_UNITS"         ,  KNOB4_COMPUTE_UNITS          )

   

    #prob.addConstraint(lambda a1, a2: not ((a1==1) and (a2==1)), ["KNOB_XYZ_TYPE", "KNOB_ENTRYID_TYPE"])
    #prob.addConstraint(lambda a1, a2, a3: not ((a1>1) and (a2==0) and (a3==0)), ["KNOB_SIMD", "KNOB_XYZ_TYPE", "KNOB_ENTRYID_TYPE"])
    #prob.addConstraint(lambda a1, a2: not ((a1>1) and (a2<2)), ["KNOB_COMPUTE_UNITS", "KNOB_ENTRYID_TYPE"])
    #prob.addConstraint(lambda a1, a2: not ((a1!=1) and (a2!=KNOB_ENTRYID_NUM_WORK_ITEMS[0])), ["KNOB_ENTRYID_TYPE", "KNOB_ENTRYID_NUM_WORK_ITEMS"])
    #prob.addConstraint(lambda a1, a2: not ((a1==0) and (a2!=0)), ["KNOB_HARDCODE_DEPTH_SIZE", "KNOB_DEPTH_LOCAL"])
    #prob.addConstraint(lambda a1, a2: not ((a1!=1) and (a2!=0)), ["KNOB_XYZ_FLATTEN_LOOP", "KNOB_XYZ_TYPE"])
    #prob.addConstraint(lambda a1, a2: not ((a1>1) and (a2!=0)), ["KNOB_UNROLL_XYZ", "KNOB_XYZ_TYPE"])
    #prob.addConstraint(lambda a1, a2: not ((a1>1) and (a2!=0)), ["KNOB_UNROLL_ENTRYID", "KNOB_ENTRYID_NUM_WORK_ITEMS"])
    #prob.addConstraint(lambda a1, a2, a3: not ((a1>1) and ((a2!=0) or (a3!=0))), ["KNOB_UNROLL_VOXEL_BLOCK", "KNOB_XYZ_TYPE", "KNOB_VOXEL_BLOCK_MEM"])
    #prob.addConstraint(lambda a1, a2, a3: not ((a1>1) and ((a2!=0) or (a3!=0))), ["KNOB_UNROLL_VOXEL_BLOCK_OUT", "KNOB_XYZ_TYPE", "KNOB_VOXEL_BLOCK_MEM"])

    #prob.addConstraint(lambda a1, a2, a3: not ((a1!=2) and (a2!=2) and (a3!=2)), ["KNOB4_SUM_NABLA_TYPE", "KNOB4_SUM_HESSIAN_TYPE", "KNOB4_SHIFT_REG_SIZE"])
    #prob.addConstraint(lambda a1, a2: not ((a1==1) and (a2!=0)), ["KNOB4_USE_ND_RANGE", "KNOB4_FLATTEN_LOOP"])
    #prob.addConstraint(lambda a1, a2: not ((a1==0) and (a2!=2)), ["KNOB4_USE_ND_RANGE", "KNOB4_PIPE_DEPTH"])

    #print(sum(1 for _ in prob.getSolutionIter()))

    #for i,sol in enumerate(prob.getSolutionIter()):
    #    if i%100000 == 0: print(i)


    finalCombinations = []

    for c in itertools.product(KNOB_COMBS, KNOB2_COMBS, KNOB4_COMBS):
        finalCombinations.append(list(itertools.chain(*c)))




    #finalCombinations = removeCombinations(allCombinations)
    
    print("Num combinations: " + str(len(finalCombinations)))
    #print("vs " + str(len(allCombinations)))


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






