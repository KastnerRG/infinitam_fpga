// Which kernels to enable
//#define ENABLE_KERNEL_INTEGRATEINTOSCENE_DEPTH_S
//#define ENABLE_KERNEL_RAYCAST_DEPTH_S
//#define ENABLE_KERNEL_INTEGRATEINTOSCENE_DEPTH_S_COMBINED
#define ENABLE_KERNEL_ICP

// Constants
#define LOOP      0
#define WORKITEMS 1
#define WORKGROUP 2

//--------------------------------
// Kernel-level knobs
//--------------------------------
//
//     - Kernel IntegrateIntoScene_depth_s
//
#define KNOB_SIMD                   1
#define KNOB_COMPUTE_UNITS          2
#define KNOB_XYZ_TYPE               WORKITEMS
#define KNOB_ENTRYID_TYPE           WORKGROUP
#define KNOB_ENTRYID_NUM_WORK_ITEMS 64 // only if KNOB_ENTRYID_TYPE == WORKITEMS
#define KNOB_HARDCODE_DEPTH_SIZE    0  // more a design choice than a knob, but can influence fmax
#define KNOB_VOXEL_BLOCK_MEM        0
#define KNOB_XYZ_FLATTEN_LOOP       1  // very little reason to set it to 0 I think
#define KNOB_UNROLL_XYZ             1  // only if KNOB_XYZ_TYPE == LOOP
#define KNOB_UNROLL_ENTRYID         1  // only if KNOB_ENTRYID_TYPE == LOOP
#define KNOB_UNROLL_VOXEL_BLOCK     1  // only if KNOB_XYZ_TYPE == LOOP and KNOB_VOXEL_BLOCK_MEM > 0
#define KNOB_UNROLL_VOXEL_BLOCK_OUT 1  // only if KNOB_XYZ_TYPE == LOOP and KNOB_VOXEL_BLOCK_MEM > 0
#define KNOB_DEPTH_LOCAL            0  // does not work on CPU/GPU
#define KNOB_MAX_W_CHECK_POSITION   1  // Position is 0 or 1
//
//     - Kernel Raycast_depth_s
//
#define KNOB2_INTERPOLATE           0 // Changes the accuracy of the result (a lot)
#define KNOB2_RAYCAST_REFINE        0 // Changes the accuracy of the result
#define KNOB2_USE_INDEX_CACHE       0
#define KNOB2_CACHE_HASH_ENTRY      1 // Not sure if that works < 16.0
#define KNOB2_CACHE_VOXEL_DATA      1 // Not sure if that works < 16.0
#define KNOB2_HARDCODE_IMG_SIZE     1 // More a design choice than a knob
#define KNOB2_USE_ND_RANGE          1
#define KNOB2_USE_MINMAXIMG         2 // 0 == no minmaximg, requires a hardcoded max depth;
                                      // 1 == use max depth only from minmaximg;   2 == use minmaximg
//
//     - Kernel IntegrateIntoScene_depth_s (combined)
//
#define KNOB3_PROJECTED_SDF_LOCAL    0
#define KNOB3_HARDCODE_DEPTH_SIZE    1  // more a design choice than a knob, but can influence fmax
#define KNOB3_DEPTH_LOCAL            0  // does not work on CPU/GPU
#define KNOB3_ENTRYID_TYPE           LOOP      // LOOP only
#define KNOB3_XYZ_TYPE               WORKITEMS // LOOP or WORKITEMS
#define KNOB3_VOXEL_BLOCK_MEM        0
#define KNOB3_UNROLL_ENTRYID         1  // only if KNOB3_ENTRYID_TYPE == LOOP
#define KNOB3_XYZ_FLATTEN_LOOP       1  // very little reason to set it to 0 I think
#define KNOB3_UNROLL_XYZ             1  // only if KNOB3_XYZ_TYPE == LOOP
#define KNOB3_UNROLL_VOXEL_BLOCK     1  // only if KNOB3_XYZ_TYPE == LOOP and KNOB3_VOXEL_BLOCK_MEM > 0
#define KNOB3_UNROLL_VOXEL_BLOCK_OUT 1  // only if KNOB3_XYZ_TYPE == LOOP and KNOB3_VOXEL_BLOCK_MEM > 0

//
//     - Kernel icp
//
#define KNOB4_INTERPOLATE_POINT      1 // 0: do not interpolate | 1: bilinear interpolation
#define KNOB4_INTERPOLATE_NORMAL     1 // 0: do not interpolate | 1: bilinear interpolation
#define KNOB4_UNROLL_NABLA           1 // 0: no unroll | 1: full unroll
#define KNOB4_UNROLL_HESSIAN         0 // 0: no unroll | 1: full unroll
#define KNOB4_SHORT_ITERATION_BRANCH 1 // 0: no branches, always long iteration | 1: have short iteration branches
#define KNOB4_SUM_NABLA_TYPE         2 // 0: loop | 1: unrolled loop | 2: shift register
#define KNOB4_SUM_HESSIAN_TYPE       2 // 0: loop | 1: unrolled loop | 2: shift register
#define KNOB4_SHIFT_REG_SIZE         2
#define KNOB4_FLATTEN_LOOP           0 // 0: loop x and y | 1: combined loop
#define KNOB4_USE_ND_RANGE           0 // 0: single WI and loop | 1: Multiple WI with second kernel
#define KNOB4_PIPE_DEPTH             2
#define KNOB4_COMPUTE_UNITS          1


//--------------------------------
// Internal logic (values can be modified)
//--------------------------------
#if KNOB_HARDCODE_DEPTH_SIZE == 0
	#define DEPTH_IMG_SIZE_X (depthImgSize.x)
	#define DEPTH_IMG_SIZE_Y (depthImgSize.y)
#else
	#define DEPTH_IMG_SIZE_X 320
	#define DEPTH_IMG_SIZE_Y 180
#endif

#if KNOB2_HARDCODE_IMG_SIZE
	#define IMGSIZE_X 320
	#define IMGSIZE_Y 180
#else
	#define IMGSIZE_X (imgSize.x)
	#define IMGSIZE_Y (imgSize.y)
#endif

#if KNOB2_USE_MINMAXIMG == 0
	#define RAYCAST_MAX_DEPTH 5.f // If raycasting is used for tracking only, we can put the max range of the sensor
#else
	#define RAYCAST_MAX_DEPTH viewFrustum_minmax.y
#endif

#if KNOB3_HARDCODE_DEPTH_SIZE == 0
	#define DEPTH_IMG_SIZE_X3 (depthImgSize.x)
	#define DEPTH_IMG_SIZE_Y3 (depthImgSize.y)
#else
	#define DEPTH_IMG_SIZE_X3 320
	#define DEPTH_IMG_SIZE_Y3 180
#endif

#define WORK_GROUP_SIZE_X4 320
#define WORK_GROUP_SIZE_Y4 180
#define WORK_GROUP_SIZE_Z4 1

//--------------------------------
// Internal logic (do not modify)
//--------------------------------
//
//     - Kernel IntegrateIntoScene_depth_s
//
#if KNOB_XYZ_TYPE == WORKITEMS
	#define WORK_GROUP_SIZE_X SDF_BLOCK_SIZE
	#define WORK_GROUP_SIZE_Y SDF_BLOCK_SIZE
	#define WORK_GROUP_SIZE_Z SDF_BLOCK_SIZE
#elif KNOB_ENTRYID_TYPE == WORKITEMS
	#define WORK_GROUP_SIZE_X KNOB_ENTRYID_NUM_WORK_ITEMS
	#define WORK_GROUP_SIZE_Y 1
	#define WORK_GROUP_SIZE_Z 1
#else
	#define WORK_GROUP_SIZE_X 1
	#define WORK_GROUP_SIZE_Y 1
	#define WORK_GROUP_SIZE_Z 1
#endif

#if (KNOB_XYZ_TYPE == WORKITEMS) && (KNOB_ENTRYID_TYPE == WORKITEMS)
	#error "EntryId and x,y,x cannot be both on work-items"
#endif

#if KNOB_ENTRYID_TYPE == LOOP
	#define ENTRYID_CONTINUE continue
	#define XYZ_CONTINUE     continue
#else
	#define ENTRYID_CONTINUE return
	#if KNOB_XYZ_TYPE == LOOP
		#define XYZ_CONTINUE continue
	#else
		#define XYZ_CONTINUE return
	#endif
#endif

#if KNOB_XYZ_TYPE != LOOP
	#define VOXELS_MEM_TYPE local
#else
	#define VOXELS_MEM_TYPE private
#endif

#if KNOB_VOXEL_BLOCK_MEM == 1
	#define VOXELS_LOCAL_OUT voxels_local
#elif KNOB_VOXEL_BLOCK_MEM == 2
	#define VOXELS_LOCAL_OUT voxels_local2
#endif

#if KNOB_DEPTH_LOCAL != 0
	#if KNOB_HARDCODE_DEPTH_SIZE == 0
		#error "Conditions not met to use local depth"
	#endif
	#define DEPTH_LOCAL depth_local
#else
	#define DEPTH_LOCAL depth
#endif

//
//     - Kernel Raycast_depth_s
//
#if KNOB2_USE_INDEX_CACHE
	#define PASS_INDEX_CACHE     , cache
	#define PASS_INDEX_CACHE_PTR , &cache
	#define ARG_INDEX_CACHE      , IndexCache * cache
#else
	#define PASS_INDEX_CACHE
	#define PASS_INDEX_CACHE_PTR
	#define ARG_INDEX_CACHE
#endif

#if KNOB2_CACHE_HASH_ENTRY
	#define VOLATILE1
#else
	#define VOLATILE1 volatile
#endif

#if KNOB2_CACHE_VOXEL_DATA
	#define VOLATILE2
#else
	#define VOLATILE2 volatile
#endif

#if KNOB2_USE_ND_RANGE
	#define WORK_GROUP_SIZE_X2 320
	#define WORK_GROUP_SIZE_Y2 180
	#define WORK_GROUP_SIZE_Z2 1
	#define WORK_GROUP_SIZE_X2_NONFPGA 32 // For non-FPGA devices
	#define WORK_GROUP_SIZE_Y2_NONFPGA 18 // For non-FPGA devices
#else
	#define WORK_GROUP_SIZE_X2 1
	#define WORK_GROUP_SIZE_Y2 1
	#define WORK_GROUP_SIZE_Z2 1
#endif

//
//     - Kernel IntegrateIntoScene_depth_s (combined)
//
#if KNOB3_DEPTH_LOCAL != 0
	#if KNOB3_HARDCODE_DEPTH_SIZE == 0
		#error "Conditions not met to use local depth"
	#endif
	#define DEPTH_LOCAL3 depth_local
#else
	#define DEPTH_LOCAL3 depth
#endif

#if KNOB3_XYZ_TYPE == WORKITEMS
	#define WORK_GROUP_SIZE_X3 SDF_BLOCK_SIZE
	#define WORK_GROUP_SIZE_Y3 SDF_BLOCK_SIZE
	#define WORK_GROUP_SIZE_Z3 SDF_BLOCK_SIZE
#elif KNOB3_ENTRYID_TYPE == WORKITEMS
#error "At this time, entryId cannot e on work-items"
#else
	#define WORK_GROUP_SIZE_X3 1
	#define WORK_GROUP_SIZE_Y3 1
	#define WORK_GROUP_SIZE_Z3 1
#endif

#if (KNOB3_XYZ_TYPE == WORKITEMS) && (KNOB3_ENTRYID_TYPE == WORKITEMS)
	#error "EntryId and x,y,x cannot be both on work-items"
#endif

#if KNOB3_ENTRYID_TYPE == LOOP
	#define ENTRYID_CONTINUE3 continue
	#define XYZ_CONTINUE3     continue
#else
	#define ENTRYID_CONTINUE3 return
	#if KNOB3_XYZ_TYPE == LOOP
		#define XYZ_CONTINUE3 continue
	#else
		#define XYZ_CONTINUE3 return
	#endif
#endif

#if KNOB3_XYZ_TYPE != LOOP
	#define VOXELS_MEM_TYPE3 local
#else
	#define VOXELS_MEM_TYPE3 private
#endif

#if KNOB3_VOXEL_BLOCK_MEM == 1
	#define VOXELS_LOCAL_OUT3 voxels_local
#elif KNOB3_VOXEL_BLOCK_MEM == 2
	#define VOXELS_LOCAL_OUT3 voxels_local2
#endif


//
//     - Kernel icp
//
#if KNOB4_USE_ND_RANGE == 1 && KNOB4_FLATTEN_LOOP == 1
#error "If using ND range, flatten loop must be off."
#endif





