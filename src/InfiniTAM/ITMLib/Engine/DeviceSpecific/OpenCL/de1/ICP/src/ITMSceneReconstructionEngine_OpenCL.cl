#include "ITMSceneReconstructionEngine_OpenCL_knobs.h"



//--------------------------------
// Application-level knobs (WARNING: duplicates from header files in the application)
//--------------------------------
#define SDF_BLOCK_SIZE          8
#define SDF_BLOCK_SIZE3         512
#define SDF_LOCAL_BLOCK_NUM     0x8000
#define SDF_TRANSFER_BLOCK_NUM  0x1000
#define SDF_BUCKET_NUM          0x10000
#define SDF_HASH_MASK           (SDF_BUCKET_NUM-1)
#define SDF_EXCESS_LIST_SIZE    0x4000
#define minmaximg_subsample_log 3 // minmaximg_subsample == 8
#define SDF_INITIAL_VALUE       32767
#define hashIndex_cl(blockPos) \
	((((unsigned int)blockPos.x * 73856093u) ^ ((unsigned int)blockPos.y * 19349669u) ^ ((unsigned int)blockPos.z * 83492791u)) \
	 & (unsigned int)SDF_HASH_MASK)





// Matrix-vector multiply (4x4)
#define matVecMul(m, vec) ((float4) (dot((m).data0, (vec)), dot((m).data1, (vec)), dot((m).data2, (vec)), dot((m).data3, (vec))))
#define matVecMul3(m, vec) ((float3) (dot((m).data0, (vec)), dot((m).data1, (vec)), dot((m).data2, (vec))))
#define matVecMul3_2(m, vec) ((float4) (dot((m).data0, (vec)), dot((m).data1, (vec)), dot((m).data2, (vec)), 1.f))



//#define matVecMul(m, vec) \
//	((float4)((m).data0.x * (vec).x, (m).data1.x * (vec).x, (m).data2.x * (vec).x, (m).data3.x * (vec).x) + \
//	 (float4)((m).data0.y * (vec).y, (m).data1.y * (vec).y, (m).data2.y * (vec).y, (m).data3.y * (vec).y) + \
//	 (float4)((m).data0.z * (vec).z, (m).data1.z * (vec).z, (m).data2.z * (vec).z, (m).data3.z * (vec).z) + \
//	 (float4)((m).data0.w * (vec).w, (m).data1.w * (vec).w, (m).data2.w * (vec).w, (m).data3.w * (vec).w))

#define mylength(vec) (sqrt(dot(vec, vec)))
#define length3(vec) (sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z))

#define IS_EQUAL3(a,b) (((a).x == (b).x) && ((a).y == (b).y) && ((a).z == (b).z))
#define TO_FLOAT3(v) ((float3)(v.x, v.y, v.z))

typedef struct
{
	short sdf;
	uchar w_depth;

} ITMVoxel_s;

typedef struct
{
	struct { short x,y,z; } pos;
	int offset;
	int ptr;

} ITMHashEntry;

typedef struct
{
	struct { int x,y,z; } blockPos;
	int blockPtr;

} IndexCache;

typedef struct
{
	float4 data0;
	float4 data1;
	float4 data2;
	float4 data3;

} Mat44_cl;


inline float SDF_shortToFloat(short x) { return (float)(x) / (float)SDF_INITIAL_VALUE; }
inline short SDF_floatToShort(float x) { return (short)((x) * (float)SDF_INITIAL_VALUE); }
#define voxel_s_initialValue(v) {v.sdf = SDF_INITIAL_VALUE; v.w_depth = 0; }


// ***************************************************************************

#ifdef ENABLE_KERNEL_INTEGRATEINTOSCENE_DEPTH_S

#ifdef ALTERA_CL
__attribute__ ((reqd_work_group_size(WORK_GROUP_SIZE_X, WORK_GROUP_SIZE_Y, WORK_GROUP_SIZE_Z)))
__attribute__ ((num_simd_work_items(KNOB_SIMD)))
__attribute__ ((num_compute_units(KNOB_COMPUTE_UNITS)))
#endif
kernel void IntegrateIntoScene_depth_s(
		int2 depthImgSize,
		float voxelSize,
		constant const Mat44_cl* restrict M_d,
		float4 projParams_d,
		float mu,
		int maxW,
		global const float* restrict depth,
		global ITMVoxel_s* restrict localVBA,
		global const ITMHashEntry* restrict hashTable,
		global const int* restrict visibleEntryIds,
		int noVisibleEntries,
		uchar stopIntegratingAtMaxW
		)
{

#if KNOB_DEPTH_LOCAL != 0
	local float depth_local[DEPTH_IMG_SIZE_X*DEPTH_IMG_SIZE_Y];

	#if KNOB_ENTRYID_TYPE != LOOP || KNOB_XYZ_TYPE != LOOP
	int flat_id  = get_local_id(0) + get_local_id(1) * get_local_size(0) + get_local_id(2) * get_local_size(0) * get_local_size(1);
	int total_wi = get_local_size(0)*get_local_size(1)*get_local_size(2);
	#else
	int flat_id = 0, total_wi = 1;
	#endif

	for(int i = flat_id; i < DEPTH_IMG_SIZE_X*DEPTH_IMG_SIZE_Y; i+=total_wi)
	{
		depth_local[i] = depth[i];
	}

	#if KNOB_ENTRYID_TYPE != LOOP || KNOB_XYZ_TYPE != LOOP
	barrier(CLK_LOCAL_MEM_FENCE);
	#endif
#endif
	


#if KNOB_VOXEL_BLOCK_MEM > 0
	VOXELS_MEM_TYPE ITMVoxel_s voxels_local[SDF_BLOCK_SIZE3];
#if KNOB_VOXEL_BLOCK_MEM > 1
	VOXELS_MEM_TYPE ITMVoxel_s VOXELS_LOCAL_OUT[SDF_BLOCK_SIZE3];
#endif
#endif


	// *** Loop entryId
	//
#if   KNOB_ENTRYID_TYPE == WORKGROUP
	const int entryId = get_group_id(0);
#elif KNOB_ENTRYID_TYPE == WORKITEMS
	const int entryId = get_local_id(0) + KNOB_ENTRYID_NUM_WORK_ITEMS * get_group_id(0);
	if(entryId >= noVisibleEntries){ return; }
#elif KNOB_ENTRYID_TYPE == LOOP
	#pragma unroll KNOB_UNROLL_ENTRYID
	for (int entryId = 0; entryId < noVisibleEntries; entryId++)
#endif
	{
		const ITMHashEntry currentHashEntry = hashTable[visibleEntryIds[entryId]];
	
		// AFAIK this check can be disabled.
		// It SHOULD be a precondition.
		//
		//if (currentHashEntry.ptr < 0) { return; ENTRYID_CONTINUE; }

	
		int3 globalPos;
		globalPos.x = currentHashEntry.pos.x;
		globalPos.y = currentHashEntry.pos.y;
		globalPos.z = currentHashEntry.pos.z;
		globalPos *= SDF_BLOCK_SIZE;
	
		global ITMVoxel_s *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
	
	
		// *** Loop x,y,z
	
#if KNOB_XYZ_TYPE == WORKITEMS
		const int x       = get_local_id(0);
		const int y       = get_local_id(1);
		const int z       = get_local_id(2);

		int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	#if KNOB_VOXEL_BLOCK_MEM > 0
		voxels_local[locId] = localVoxelBlock[locId];
	#endif
	#if KNOB_VOXEL_BLOCK_MEM > 1
		VOXELS_LOCAL_OUT[locId] = localVoxelBlock[locId];
	#endif


#elif KNOB_XYZ_TYPE == LOOP

	#if KNOB_VOXEL_BLOCK_MEM > 0
		#pragma unroll KNOB_UNROLL_VOXEL_BLOCK
		for(int locId = 0; locId < SDF_BLOCK_SIZE3; locId++)
		{
			voxels_local[locId] = localVoxelBlock[locId];
		#if KNOB_VOXEL_BLOCK_MEM > 1
			VOXELS_LOCAL_OUT[locId] = localVoxelBlock[locId];
		#endif
		}
	#endif

	#if KNOB_XYZ_FLATTEN_LOOP == 0
		for (int locId = 0, z = 0; z < SDF_BLOCK_SIZE; z++)
			for (int y = 0; y < SDF_BLOCK_SIZE; y++)
				#pragma unroll KNOB_UNROLL_XYZ
				for (int x = 0; x < SDF_BLOCK_SIZE; x++, locId++)
	#else
		int x = -1, y = 0, z = 0;
		
		#pragma unroll KNOB_UNROLL_XYZ
		for(int locId = 0; locId < SDF_BLOCK_SIZE3; locId++)
	#endif
#endif
		{

			// Case 1: Get voxel here and check for max weight
#if KNOB_MAX_W_CHECK_POSITION == 0
	#if KNOB_VOXEL_BLOCK_MEM > 0
			ITMVoxel_s voxel = voxels_local[locId];
	#else
			ITMVoxel_s voxel = localVoxelBlock[locId];
	#endif
			if (stopIntegratingAtMaxW) if (voxel.w_depth == maxW) XYZ_CONTINUE;
#endif



#if KNOB_XYZ_TYPE == LOOP && KNOB_XYZ_FLATTEN_LOOP != 0
			x++;
			if(x >= SDF_BLOCK_SIZE){ x = 0; y++; }
			if(y >= SDF_BLOCK_SIZE){ y = 0; z++; }
#endif

			float4 pt_model;
			pt_model.x = (float)(globalPos.x + x) * voxelSize;
			pt_model.y = (float)(globalPos.y + y) * voxelSize;
			pt_model.z = (float)(globalPos.z + z) * voxelSize;
			pt_model.w = 1.0f;


			// Project point into image
			float4 pt_camera = matVecMul(*M_d, pt_model);
			if (pt_camera.z <= 0) { XYZ_CONTINUE; }

			float2 pt_image;
			pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
			pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
			if ((pt_image.x < 1) || (pt_image.x > DEPTH_IMG_SIZE_X - 2) || (pt_image.y < 1) || (pt_image.y > DEPTH_IMG_SIZE_Y - 2)) XYZ_CONTINUE;

			// Get measured depth from image
			float depth_measure = DEPTH_LOCAL[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * DEPTH_IMG_SIZE_X];
			if (depth_measure <= 0.0) XYZ_CONTINUE;

			// Check whether voxel needs updating
			float eta = depth_measure - pt_camera.z;
			if (eta < -mu) XYZ_CONTINUE;

			// Case 2: Get voxel here and check for max weight
#if KNOB_MAX_W_CHECK_POSITION > 0
	#if KNOB_VOXEL_BLOCK_MEM > 0
			ITMVoxel_s voxel = voxels_local[locId];
	#else
			ITMVoxel_s voxel = localVoxelBlock[locId];
	#endif
			if (stopIntegratingAtMaxW) if (voxel.w_depth == maxW) XYZ_CONTINUE;
#endif

			// Compute updated SDF value and reliability
			float oldF = SDF_shortToFloat(voxel.sdf);
			int   oldW = voxel.w_depth;

			float newF = min(1.0f, eta / mu);
			int   newW = 1;

			newF = oldW * oldF + newW * newF;
			newW = oldW + newW;
			newF /= newW;
			newW = min(newW, maxW);

			// Write back
#if KNOB_VOXEL_BLOCK_MEM > 0
			VOXELS_LOCAL_OUT[locId].sdf     = SDF_floatToShort(newF);
			VOXELS_LOCAL_OUT[locId].w_depth = newW;
#else
			localVoxelBlock[locId].sdf     = SDF_floatToShort(newF);
			localVoxelBlock[locId].w_depth = newW;
#endif

		} // for x,y,z
	


#if KNOB_VOXEL_BLOCK_MEM > 0
#if KNOB_XYZ_TYPE == WORKITEMS
		localVoxelBlock[locId] = VOXELS_LOCAL_OUT[locId];
#elif KNOB_XYZ_TYPE == LOOP	
		#pragma unroll KNOB_UNROLL_VOXEL_BLOCK_OUT
		for(int locId = 0; locId < SDF_BLOCK_SIZE3; locId++)
		{
			localVoxelBlock[locId] = VOXELS_LOCAL_OUT[locId];
		}
#endif
#endif


	} // for entryId
}


#endif // ENABLE_KERNEL_INTEGRATEINTOSCENE_DEPTH_S




// ***************************************************************************

#ifdef ENABLE_KERNEL_RAYCAST_DEPTH_S

ITMVoxel_s readVoxel_s(
		global const ITMVoxel_s*   restrict voxelData,
		global const ITMHashEntry* restrict voxelIndex,
		int3  point,
		bool * isFound
		ARG_INDEX_CACHE)
{
	int3 blockPos;

	blockPos.x = ((point.x < 0) ? point.x - SDF_BLOCK_SIZE + 1 : point.x) / SDF_BLOCK_SIZE;
	blockPos.y = ((point.y < 0) ? point.y - SDF_BLOCK_SIZE + 1 : point.y) / SDF_BLOCK_SIZE;
	blockPos.z = ((point.z < 0) ? point.z - SDF_BLOCK_SIZE + 1 : point.z) / SDF_BLOCK_SIZE;

	int linearIdx = point.x + (point.y - blockPos.x) * SDF_BLOCK_SIZE + (point.z - blockPos.y) * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE - blockPos.z * SDF_BLOCK_SIZE3;

#if KNOB2_USE_INDEX_CACHE
	if IS_EQUAL3(blockPos, cache->blockPos)
	{
		*isFound = true;
		return voxelData[cache->blockPtr + linearIdx];
	}
#endif

	int hashIdx = hashIndex_cl(blockPos);

	while (true)
	{
		VOLATILE1 ITMHashEntry hashEntry = voxelIndex[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0)
		{
			*isFound = true;
#if KNOB2_USE_INDEX_CACHE
			cache->blockPos.x = blockPos.x;
			cache->blockPos.y = blockPos.y;
			cache->blockPos.z = blockPos.z;
			cache->blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
			VOLATILE2 ITMVoxel_s res = voxelData[cache->blockPtr + linearIdx];
			return res;
#else
			VOLATILE2 ITMVoxel_s res = voxelData[hashEntry.ptr * SDF_BLOCK_SIZE3 + linearIdx];
			return res;
#endif
		}

		if (hashEntry.offset < 1) break;
		hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
	}

	*isFound = false;
	ITMVoxel_s empty; voxel_s_initialValue(empty);
	return empty;
}

float readFromSDF_float_uninterpolated_s(
		global const ITMVoxel_s*   restrict voxelData,
		global const ITMHashEntry* restrict voxelIndex,
		float3 point,
		bool * isFound
		ARG_INDEX_CACHE)
{
	ITMVoxel_s res = readVoxel_s(voxelData, voxelIndex, (int3)(round(point.x), round(point.y), round(point.z)), isFound
			PASS_INDEX_CACHE);
	return SDF_shortToFloat(res.sdf);
}


#if KNOB2_INTERPOLATE
float readFromSDF_float_interpolated_s(
		global const ITMVoxel_s*   restrict voxelData,
		global const ITMHashEntry* restrict voxelIndex,
		float3 point,
		bool * isFound
		ARG_INDEX_CACHE)
{
	float  res1, res2, v1, v2;

	float3 intFloor = (float3)(floor(point.x), floor(point.y), floor(point.z));
	float3 coeff    = point - intFloor;
	int3   pos      = (int3)((int)intFloor.x, (int)intFloor.y, (int)intFloor.z);


	v1 = readVoxel_s(voxelData, voxelIndex, pos + (int3)(0, 0, 0), isFound  PASS_INDEX_CACHE).sdf;
	v2 = readVoxel_s(voxelData, voxelIndex, pos + (int3)(1, 0, 0), isFound  PASS_INDEX_CACHE).sdf;
	res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	v1 = readVoxel_s(voxelData, voxelIndex, pos + (int3)(0, 1, 0), isFound  PASS_INDEX_CACHE).sdf;
	v2 = readVoxel_s(voxelData, voxelIndex, pos + (int3)(1, 1, 0), isFound  PASS_INDEX_CACHE).sdf;
	res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	v1 = readVoxel_s(voxelData, voxelIndex, pos + (int3)(0, 0, 1), isFound  PASS_INDEX_CACHE).sdf;
	v2 = readVoxel_s(voxelData, voxelIndex, pos + (int3)(1, 0, 1), isFound  PASS_INDEX_CACHE).sdf;
	res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	v1 = readVoxel_s(voxelData, voxelIndex, pos + (int3)(0, 1, 1), isFound  PASS_INDEX_CACHE).sdf;
	v2 = readVoxel_s(voxelData, voxelIndex, pos + (int3)(1, 1, 1), isFound  PASS_INDEX_CACHE).sdf;
	res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	*isFound = true;
	return SDF_shortToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}
#else
#define readFromSDF_float_interpolated_s readFromSDF_float_uninterpolated_s
#endif


#ifdef ALTERA_CL
	#if KNOB2_USE_ND_RANGE
__attribute__ ((reqd_work_group_size(WORK_GROUP_SIZE_X2, WORK_GROUP_SIZE_Y2, WORK_GROUP_SIZE_Z2)))
__attribute__ ((num_simd_work_items(1)))
__attribute__ ((num_compute_units(1)))
	#else
__attribute__((max_global_work_dim(0)))
	#endif
#endif
kernel void Raycast_depth_s(
		global       float4*       restrict pointsRay,
		global const ITMVoxel_s*   restrict voxelData,
		global const ITMHashEntry* restrict voxelIndex,
		global const float2*       restrict minmaximg,
		int2   imgSize,
		//float  mu,
		float  oneOverVoxelSize,
		float  stepScale,
		float4 projParams,
		constant const Mat44_cl* restrict invM
		)
{
#if KNOB2_USE_ND_RANGE
	int x = get_global_id(0);
	int y = get_global_id(1);

	if (x >= IMGSIZE_X || y >= IMGSIZE_Y) return;

	int locId = x + y * IMGSIZE_X;
	{
#else
	int x = -1, y = 0;
	for(int locId = 0; locId < IMGSIZE_X*IMGSIZE_Y; locId++)
	{
		x++;
		if(x >= IMGSIZE_X){ x = 0; y++; }
#endif

#if KNOB2_USE_MINMAXIMG > 0
		//int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * IMGSIZE_X;
		int locId2 = (x >> minmaximg_subsample_log) + (y >> minmaximg_subsample_log) * IMGSIZE_X;
 
		float2  viewFrustum_minmax = minmaximg[locId2];
#endif

		//float stepScale = mu * oneOverVoxelSize;


		float4 pt_camera_f;

#if KNOB2_USE_MINMAXIMG == 2
		pt_camera_f.z = viewFrustum_minmax.x;
		pt_camera_f.x = pt_camera_f.z * (((float)x - projParams.z) * projParams.x);
		pt_camera_f.y = pt_camera_f.z * (((float)y - projParams.w) * projParams.y);
		pt_camera_f.w = 1.0f;
		float  totalLength = length3(pt_camera_f) * oneOverVoxelSize;
		float3 pt_block_s  = matVecMul3(*invM, pt_camera_f) * oneOverVoxelSize;
#else
		float  totalLength = 0.f;
		float3 pt_block_s  = (float3)(invM->data0.w, invM->data1.w, invM->data2.w) * oneOverVoxelSize;
#endif
 
		pt_camera_f.z = RAYCAST_MAX_DEPTH;
		pt_camera_f.x = pt_camera_f.z * (((float)x - projParams.z) * projParams.x);
		pt_camera_f.y = pt_camera_f.z * (((float)y - projParams.w) * projParams.y);
		pt_camera_f.w = 1.0f;
		float  totalLengthMax = length3(pt_camera_f) * oneOverVoxelSize;
		float3 pt_block_e     = matVecMul3(*invM, pt_camera_f) * oneOverVoxelSize;

		float3 rayDirection   = pt_block_e - pt_block_s;
		float  direction_norm = rsqrt(rayDirection.x * rayDirection.x + rayDirection.y * rayDirection.y + rayDirection.z * rayDirection.z);
		rayDirection *= direction_norm;

#if KNOB2_USE_INDEX_CACHE
		IndexCache cache;
#endif
		float3     pt_result = pt_block_s;
		float      sdfValue = 1.0f;
		float      stepLength;
		bool       hash_found;

		while (totalLength < totalLengthMax)
		{
			sdfValue = readFromSDF_float_uninterpolated_s(voxelData, voxelIndex, pt_result, &hash_found  PASS_INDEX_CACHE_PTR);

			if (!hash_found){ stepLength = SDF_BLOCK_SIZE; }
			else
			{
#if KNOB2_INTERPOLATE
				if ((sdfValue <= 0.1f) && (sdfValue >= -0.5f))
				{
					sdfValue = readFromSDF_float_interpolated_s(voxelData, voxelIndex, pt_result, &hash_found  PASS_INDEX_CACHE_PTR);
				}
#endif
				if (sdfValue <= 0.0f) break;
				stepLength = max(sdfValue * stepScale, 1.0f);
			}

			pt_result += stepLength * rayDirection;
			totalLength += stepLength;
		}

		bool pt_found;
		if (sdfValue <= 0.0f)
		{
			stepLength = sdfValue * stepScale;
			pt_result += stepLength * rayDirection;

#if KNOB2_RAYCAST_REFINE
			sdfValue = readFromSDF_float_interpolated_s(voxelData, voxelIndex, pt_result, &hash_found  PASS_INDEX_CACHE_PTR);
			stepLength = sdfValue * stepScale;
			pt_result += stepLength * rayDirection;
#endif

			pt_found = true;
		}
		else pt_found = false;

		pointsRay[locId].x = pt_result.x;
		pointsRay[locId].y = pt_result.y;
		pointsRay[locId].z = pt_result.z;
		if (pt_found) pointsRay[locId].w = 1.0f; else pointsRay[locId].w = 0.0f;

	
	}

}

#endif // ENABLE_KERNEL_RAYCAST_DEPTH_S



// ***************************************************************************

#ifdef ENABLE_KERNEL_INTEGRATEINTOSCENE_DEPTH_S_COMBINED

#ifdef ALTERA_CL
__attribute__ ((reqd_work_group_size(WORK_GROUP_SIZE_X3, WORK_GROUP_SIZE_Y3, WORK_GROUP_SIZE_Z3)))
__attribute__ ((num_simd_work_items(1)))
__attribute__ ((num_compute_units(1)))
#endif
kernel void IntegrateRaycastCombined_depth_s(
		int2 depthImgSize,
		float voxelSize,
		constant const Mat44_cl* restrict M_d,
		float4 projParams_d,
		float mu,
		int maxW,
		global const float* restrict depth,
		global ITMVoxel_s* restrict localVBA,
		global const ITMHashEntry* restrict hashTable,
		global const int* restrict visibleEntryIds,
		int noVisibleEntries,
		uchar stopIntegratingAtMaxW,
		global float4* restrict pointsRay,
		float oneOverVoxelSize,
		constant const Mat44_cl* restrict invM_d
#if !KNOB3_PROJECTED_SDF_LOCAL
		, global short* restrict projectedSdf
#endif
		)
{
#if KNOB3_DEPTH_LOCAL || KNOB3_PROJECTED_SDF_LOCAL
	// Initialize local buffers

	#if KNOB3_ENTRYID_TYPE != LOOP || KNOB3_XYZ_TYPE != LOOP
	int flat_id  = get_local_id(0) + get_local_id(1) * get_local_size(0) + get_local_id(2) * get_local_size(0) * get_local_size(1);
	int total_wi = get_local_size(0)*get_local_size(1)*get_local_size(2);
	#else
	int flat_id = 0, total_wi = 1;
	#endif


	#if KNOB3_PROJECTED_SDF_LOCAL
	local short projectedSdf[DEPTH_IMG_SIZE_X3*DEPTH_IMG_SIZE_Y3];
	#endif

	#if KNOB3_DEPTH_LOCAL
	local float depth_local[DEPTH_IMG_SIZE_X3*DEPTH_IMG_SIZE_Y3];
	#endif


	for(int i = flat_id; i < DEPTH_IMG_SIZE_X3*DEPTH_IMG_SIZE_Y3; i+=total_wi)
	{
	#if KNOB3_DEPTH_LOCAL
		depth_local[i] = depth[i];
	#endif
	#if KNOB3_PROJECTED_SDF_LOCAL
		projectedSdf[i] = SDF_INITIAL_VALUE;
	#endif
	}
 
	#if (KNOB3_ENTRYID_TYPE != LOOP || KNOB3_XYZ_TYPE != LOOP)
	barrier(CLK_LOCAL_MEM_FENCE);
	#endif
#endif
	


#if KNOB3_VOXEL_BLOCK_MEM > 0
	VOXELS_MEM_TYPE3 ITMVoxel_s voxels_local[SDF_BLOCK_SIZE3];
#if KNOB3_VOXEL_BLOCK_MEM > 1
	VOXELS_MEM_TYPE3 ITMVoxel_s VOXELS_LOCAL_OUT3[SDF_BLOCK_SIZE3];
#endif
#endif


	// *** Loop entryId
	//
//#if   KNOB3_ENTRYID_TYPE == WORKGROUP
//	const int entryId = get_group_id(0);
//#elif KNOB3_ENTRYID_TYPE == WORKITEMS
//	const int entryId = get_local_id(0) + KNOB3_ENTRYID_NUM_WORK_ITEMS * get_group_id(0);
//	if(entryId >= noVisibleEntries){ return; }
//#elif   KNOB3_ENTRYID_TYPE == WORKITEMS
//	const int entryId = get_local_id(0);
#if KNOB3_ENTRYID_TYPE == LOOP
	#pragma unroll KNOB3_UNROLL_ENTRYID
	for (int entryId = 0; entryId < noVisibleEntries; entryId++)
#endif
	{
		const ITMHashEntry currentHashEntry = hashTable[visibleEntryIds[entryId]];
	
		// AFAIK this check can be disabled.
		// It SHOULD be a precondition.
		//
		//if (currentHashEntry.ptr < 0) { return; ENTRYID_CONTINUE3; }

	
		int3 globalPos;
		globalPos.x = currentHashEntry.pos.x;
		globalPos.y = currentHashEntry.pos.y;
		globalPos.z = currentHashEntry.pos.z;
		globalPos *= SDF_BLOCK_SIZE;
	
		global ITMVoxel_s *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
	
	
		// *** Loop x,y,z
	
#if KNOB3_XYZ_TYPE == WORKITEMS
		const int x       = get_local_id(0);
		const int y       = get_local_id(1);
		const int z       = get_local_id(2);

		int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	#if KNOB3_VOXEL_BLOCK_MEM > 0
		voxels_local[locId] = localVoxelBlock[locId];
	#endif
	#if KNOB3_VOXEL_BLOCK_MEM > 1
		VOXELS_LOCAL_OUT3[locId] = localVoxelBlock[locId];
	#endif


#elif KNOB3_XYZ_TYPE == LOOP

	#if KNOB3_VOXEL_BLOCK_MEM > 0
		#pragma unroll KNOB3_UNROLL_VOXEL_BLOCK
		for(int locId = 0; locId < SDF_BLOCK_SIZE3; locId++)
		{
			voxels_local[locId] = localVoxelBlock[locId];
		#if KNOB3_VOXEL_BLOCK_MEM > 1
			VOXELS_LOCAL_OUT3[locId] = localVoxelBlock[locId];
		#endif
		}
	#endif

	#if KNOB3_XYZ_FLATTEN_LOOP == 0
		for (int locId = 0, z = 0; z < SDF_BLOCK_SIZE; z++)
			for (int y = 0; y < SDF_BLOCK_SIZE; y++)
				#pragma unroll KNOB3_UNROLL_XYZ
				for (int x = 0; x < SDF_BLOCK_SIZE; x++, locId++)
	#else
		int x = -1, y = 0, z = 0;
		
		#pragma unroll KNOB3_UNROLL_XYZ
		for(int locId = 0; locId < SDF_BLOCK_SIZE3; locId++)
	#endif
#endif
		{


#if KNOB3_VOXEL_BLOCK_MEM > 0
			ITMVoxel_s voxel = voxels_local[locId];
#else
			ITMVoxel_s voxel = localVoxelBlock[locId];
#endif



#if KNOB3_XYZ_TYPE == LOOP && KNOB3_XYZ_FLATTEN_LOOP != 0
			x++;
			if(x >= SDF_BLOCK_SIZE){ x = 0; y++; }
			if(y >= SDF_BLOCK_SIZE){ y = 0; z++; }
#endif

			float4 pt_voxel;
			pt_voxel.x = (float)(globalPos.x + x);
			pt_voxel.y = (float)(globalPos.y + y);
			pt_voxel.z = (float)(globalPos.z + z);
			pt_voxel.w = 1.0f;

			float4 pt_model;
			pt_model.x = pt_voxel.x * voxelSize;
			pt_model.y = pt_voxel.y * voxelSize;
			pt_model.z = pt_voxel.z * voxelSize;
			pt_model.w = 1.0f;


			// Project point into image
			float4 pt_camera = matVecMul(*M_d, pt_model);
			if (pt_camera.z <= 0) { XYZ_CONTINUE3; }

			float2 pt_image;
			pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
			pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
			//if ((pt_image.x < 1) || (pt_image.x > DEPTH_IMG_SIZE_X3 - 2) || (pt_image.y < 1) || (pt_image.y > DEPTH_IMG_SIZE_Y3 - 2)) XYZ_CONTINUE3;
			if ((pt_image.x < 0) || (pt_image.x > DEPTH_IMG_SIZE_X3 - 1) || (pt_image.y < 0) || (pt_image.y > DEPTH_IMG_SIZE_Y3 - 1)) XYZ_CONTINUE3;

			// Get measured depth from image
			int depthIdx = (int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * DEPTH_IMG_SIZE_X3;
			float depth_measure = DEPTH_LOCAL3[depthIdx];

			bool doUpdate = false;

			float eta = depth_measure - pt_camera.z;

			// Check whether voxel needs updating
			if (depth_measure > 0.0)
			{
				if (eta >= -mu)
				{
					if (!stopIntegratingAtMaxW){ doUpdate = true; }
					else
					{
						if (voxel.w_depth != maxW){ doUpdate = true; }
					}
				}
			}

			short oldF_s = voxel.sdf;
			float oldF   = SDF_shortToFloat(oldF_s);
			int   oldW   = voxel.w_depth;

			short newF_s = oldF_s;
			float newF   = oldF;
			int   newW   = oldW;

			if(doUpdate)
			{
				// Compute updated SDF value and reliability

				newF = min(1.0f, eta / mu);
				newW = 1;

				newF = oldW * oldF + newW * newF;
				newW = oldW + newW;
				newF /= newW;
				newW = min(newW, maxW);

				newF_s = SDF_floatToShort(newF);

				// Write back
#if KNOB3_VOXEL_BLOCK_MEM > 0
				VOXELS_LOCAL_OUT3[locId].sdf     = newF_s;
				VOXELS_LOCAL_OUT3[locId].w_depth = newW;
#else
				localVoxelBlock[locId].sdf     = newF_s;
				localVoxelBlock[locId].w_depth = newW;
#endif
			}

			if((fabs(eta) <= mu) && (abs(newF_s) < projectedSdf[depthIdx]))
			{
				float4 pt_ray = pt_camera;
				pt_ray.z += newF * mu;
				pt_ray = matVecMul(*invM_d, pt_ray);
				pt_ray *= oneOverVoxelSize;
				pt_ray.w = 1.f;

				projectedSdf[depthIdx] = abs(newF_s);
				//pointsRay[depthIdx]    = pt_voxel;
				pointsRay[depthIdx]    = pt_ray;
			}

		} // for x,y,z
	


#if KNOB3_VOXEL_BLOCK_MEM > 0
#if KNOB3_XYZ_TYPE == WORKITEMS
		localVoxelBlock[locId] = VOXELS_LOCAL_OUT3[locId];
#elif KNOB3_XYZ_TYPE == LOOP	
		#pragma unroll KNOB3_UNROLL_VOXEL_BLOCK_OUT
		for(int locId = 0; locId < SDF_BLOCK_SIZE3; locId++)
		{
			localVoxelBlock[locId] = VOXELS_LOCAL_OUT3[locId];
		}
#endif
#endif


	} // for entryId
}


#endif // ENABLE_KERNEL_INTEGRATEINTOSCENE_DEPTH_S_COMBINED




#ifdef ENABLE_KERNEL_ICP

#define HESSIAN_SIZE        21 // (6 + 5 + 4 + 3 + 2 + 1)
#define SHORT_HESSIAN_SIZE  6  //(3 + 2 + 1)

#define getValueAtPoint(source, position, sizeX) ( source[(int)round(position.x) + (int)round(position.y) * sizeX] )

#if (KNOB4_INTERPOLATE_NORMAL != 0 || KNOB4_INTERPOLATE_POINT != 0)
float4 interpolateBilinear_withHoles_OpenCL(
		global const float4* restrict source,
		float2 position,
		int2   imgSize)
{
	float4 a, b, c, d;
	float4 result;
	short2 p;
	float2 delta;

	p.x = (short)floor(position.x);
	p.y = (short)floor(position.y);
	delta.x = position.x - (float)p.x;
	delta.y = position.y - (float)p.y;

	a = source[p.x + p.y * imgSize.x];
	b = source[(p.x + 1) + p.y * imgSize.x];
	c = source[p.x + (p.y + 1) * imgSize.x];
	d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	if (a.w < 0 || b.w < 0 || c.w < 0 || d.w < 0)
	{
		result.x = 0; result.y = 0; result.z = 0; result.w = -1.0f;
		return result;
	}

	result.x = ((float)a.x * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.x * delta.x * (1.0f - delta.y) +
		(float)c.x * (1.0f - delta.x) * delta.y + (float)d.x * delta.x * delta.y);
	result.y = ((float)a.y * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.y * delta.x * (1.0f - delta.y) +
		(float)c.y * (1.0f - delta.x) * delta.y + (float)d.y * delta.x * delta.y);
	result.z = ((float)a.z * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.z * delta.x * (1.0f - delta.y) +
		(float)c.z * (1.0f - delta.x) * delta.y + (float)d.z * delta.x * delta.y);
	result.w = ((float)a.w * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.w * delta.x * (1.0f - delta.y) +
		(float)c.w * (1.0f - delta.x) * delta.y + (float)d.w * delta.x * delta.y);

	return result;
}
#endif

#ifdef ALTERA_CL
#if KNOB4_USE_ND_RANGE == 0
__attribute__((max_global_work_dim(0)))
#else
__attribute__ ((reqd_work_group_size(WORK_GROUP_SIZE_X4, WORK_GROUP_SIZE_Y4, WORK_GROUP_SIZE_Z4)))
#endif
__attribute__ ((num_simd_work_items(1)))
__attribute__ ((num_compute_units(KNOB4_COMPUTE_UNITS)))
#endif
kernel void icp(
		global const float* restrict depth,
		int2   viewImageSize,
		float4 viewIntrinsics,
		constant const Mat44_cl* restrict approxInvPose,
		constant const Mat44_cl* restrict scenePose,
		int2   sceneImageSize,
		float4 sceneIntrinsics,
		global const float4* restrict pointsMap,
		global const float4* restrict normalsMap,
		float distThresh,
		uchar shortIteration,
		uchar rotationOnly,
		global float* restrict sumHessian,
		global float* restrict sumNabla,
		global int*   restrict noValidPoints,
		global float* restrict sumF
#if KNOB4_USE_ND_RANGE == 1
		,
		write_only pipe uchar __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) valid_pipe,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) sumF_pipe,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) nabla_pipe0,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) nabla_pipe1,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) nabla_pipe2,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) nabla_pipe3,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) nabla_pipe4,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) nabla_pipe5,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe0,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe1,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe2,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe3,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe4,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe5,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe6,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe7,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe8,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe9,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe10,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe11,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe12,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe13,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe14,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe15,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe16,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe17,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe18,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe19,
		write_only pipe float __attribute__((blocking))
		                      __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe20
#endif
)
{
#if KNOB4_SUM_NABLA_TYPE != 2 || KNOB4_UNROLL_NABLA == 0 || KNOB4_UNROLL_HESSIAN == 0
#if KNOB4_SHORT_ITERATION_BRANCH != 0
	const int noPara = shortIteration ? 3 : 6;
#else
	const int noPara = 6;
#endif
#endif

#if KNOB4_SUM_HESSIAN_TYPE != 2
#if KNOB4_SHORT_ITERATION_BRANCH != 0
	const int noParaSQ = shortIteration ? SHORT_HESSIAN_SIZE : HESSIAN_SIZE;
#else
	const int noParaSQ = HESSIAN_SIZE;
#endif
#endif


	int noValidPoints_local = 0;

#if KNOB4_SUM_NABLA_TYPE != 2 && KNOB4_SUM_HESSIAN_TYPE != 2
	float sumF_local        = 0.f;
#endif


	// Initialize Nabla buffer
#if KNOB4_SUM_NABLA_TYPE == 2
	float8 nabla_buffer = (float8)(0,0,0,0,0,0,0,0);
	float8 nabla_shiftreg[KNOB4_SHIFT_REG_SIZE];
	for(unsigned int i = 0; i < KNOB4_SHIFT_REG_SIZE; i++)
	{
		nabla_shiftreg[i] = nabla_buffer;
	}
#else
	float sumNabla_local[6];
	for (int i = 0; i < 6; i++) sumNabla_local[i] = 0.0f;
#endif


	// Initialize Hessian buffer
#if KNOB4_SUM_HESSIAN_TYPE == 2
	float16 hessian_buffer1 = (float16)(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
	float8  hessian_buffer2 = (float8) (0,0,0,0,0,0,0,0);
	float16 hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE];
	float8  hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE];
	for(unsigned int i = 0; i < KNOB4_SHIFT_REG_SIZE; i++)
	{
		hessian_shiftreg1[i] = hessian_buffer1;
		hessian_shiftreg2[i] = hessian_buffer2;
	}
#else
	float sumHessian_local[HESSIAN_SIZE];
	for (int i = 0; i < HESSIAN_SIZE; i++) sumHessian_local[i] = 0.0f;
#endif

	// Loop over X and Y
#if KNOB4_USE_ND_RANGE == 0
#if KNOB4_FLATTEN_LOOP != 0
	int x = -1, y = 0;
	for (int xy = 0; xy < viewImageSize.x * viewImageSize.y; xy++)
#else
	for (int y = 0; y < viewImageSize.y; y++)
		for (int x = 0; x < viewImageSize.x; x++)
#endif
#else
	uchar isValid = 0;
	float localF = 0.f;
	float localHessian[HESSIAN_SIZE] = {0};
	float localNabla[6] = {0};
	int x = get_global_id(0);
	int y = get_global_id(1);
#endif
		{
#if KNOB4_USE_ND_RANGE == 0
			float localHessian[HESSIAN_SIZE];
			float localNabla[6];
#endif

			float A[6];
			float b;

#if KNOB4_FLATTEN_LOOP != 0
			x++;
			if(x >= viewImageSize.x){ x = 0; y++; }

			float d = depth[xy];
#else
			float d = depth[x + y * viewImageSize.x];
#endif

			if (d > 1e-8f) //check if valid -- != 0.0f
			{
				float4 tmp3Dpoint, tmp3Dpoint_reproj;
				float3 ptDiff;
				float4 curr3Dpoint, corr3Dnormal;
				float2 tmp2Dpoint;

				tmp3Dpoint.x = d * (((float)x - viewIntrinsics.z) * viewIntrinsics.x); // viewIntrinsics.x is 1/fx
				tmp3Dpoint.y = d * (((float)y - viewIntrinsics.w) * viewIntrinsics.y); // viewIntrinsics.y is 1/fy
				tmp3Dpoint.z = d;
				tmp3Dpoint.w = 1.0f;

				// transform to previous frame coordinates
				tmp3Dpoint = matVecMul3_2(*approxInvPose, tmp3Dpoint);
	//			tmp3Dpoint.w = 1.0f;

				// project into previous rendered image
				tmp3Dpoint_reproj = matVecMul3_2(*scenePose, tmp3Dpoint);

				if (tmp3Dpoint_reproj.z > 0.0f)
				{
					tmp2Dpoint.x = sceneIntrinsics.x * tmp3Dpoint_reproj.x / tmp3Dpoint_reproj.z + sceneIntrinsics.z;
					tmp2Dpoint.y = sceneIntrinsics.y * tmp3Dpoint_reproj.y / tmp3Dpoint_reproj.z + sceneIntrinsics.w;

					if ((tmp2Dpoint.x >= 0.0f) && (tmp2Dpoint.x <= sceneImageSize.x - 2) && (tmp2Dpoint.y >= 0.0f) && (tmp2Dpoint.y <= sceneImageSize.y - 2))
					{

			#if KNOB4_INTERPOLATE_POINT != 0
						curr3Dpoint = interpolateBilinear_withHoles_OpenCL(pointsMap, tmp2Dpoint, sceneImageSize);
			#else
						curr3Dpoint = getValueAtPoint(pointsMap, tmp2Dpoint, sceneImageSize.x);
			#endif
						if (curr3Dpoint.w >= 0.0f)
						{

							ptDiff.x = curr3Dpoint.x - tmp3Dpoint.x;
							ptDiff.y = curr3Dpoint.y - tmp3Dpoint.y;
							ptDiff.z = curr3Dpoint.z - tmp3Dpoint.z;
							float dist = ptDiff.x * ptDiff.x + ptDiff.y * ptDiff.y + ptDiff.z * ptDiff.z;

							if (dist <= distThresh)
							{

					#if KNOB4_INTERPOLATE_POINT != 0
								corr3Dnormal = interpolateBilinear_withHoles_OpenCL(normalsMap, tmp2Dpoint, sceneImageSize);
								if(corr3Dnormal.w >= 0.0f)
					#else
								corr3Dnormal = getValueAtPoint(normalsMap, tmp2Dpoint, sceneImageSize.x);
					#endif
								{

									b = corr3Dnormal.x * ptDiff.x + corr3Dnormal.y * ptDiff.y + corr3Dnormal.z * ptDiff.z;


									if (shortIteration)
									{
										if (rotationOnly)
										{
											A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
											A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
											A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
										}
										else
										{
											A[0] = corr3Dnormal.x;
											A[1] = corr3Dnormal.y;
											A[2] = corr3Dnormal.z;
										}
									}
									else
									{
										A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
										A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
										A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
										A[3] = corr3Dnormal.x;
										A[4] = corr3Dnormal.y;
										A[5] = corr3Dnormal.z;
									}


						#if KNOB4_USE_ND_RANGE == 0
									float localF = b * b;
						#else
									localF = b * b;
						#endif

									// --------------------
									// Calculate Hessian and Nabla
									// --------------------
						#if (KNOB4_UNROLL_HESSIAN == 0)
									for (int r = 0, counter = 0; r < noPara; r++)
									{
						#if (KNOB4_UNROLL_NABLA == 0)
										localNabla[r] = b * A[r];
						#endif

										for (int c = 0; c <= r; c++, counter++)
										{
											localHessian[counter] = A[r] * A[c];
										}
									}
						#endif

						#if (KNOB4_UNROLL_NABLA != 0)
									localNabla[0] = b * A[0];
									localNabla[1] = b * A[1];
									localNabla[2] = b * A[2];
						#if (KNOB4_SHORT_ITERATION_BRANCH != 0)
									if(!shortIteration)
						#endif
									{
										localNabla[3] = b * A[3];
										localNabla[4] = b * A[4];
										localNabla[5] = b * A[5];
									}
						#endif

						#if (KNOB4_UNROLL_HESSIAN != 0)

						#if (KNOB4_UNROLL_NABLA == 0)
									for (int r = 0; r < noPara; r++)
									{
										localNabla[r] = b * A[r];
									}
						#endif

									localHessian[0] = A[0] * A[0];
									localHessian[1] = A[1] * A[0];
									localHessian[2] = A[1] * A[1];
									localHessian[3] = A[2] * A[0];
									localHessian[4] = A[2] * A[1];
									localHessian[5] = A[2] * A[2];
						#if (KNOB4_SHORT_ITERATION_BRANCH != 0)
									if(!shortIteration)
						#endif
									{
										localHessian[6] = A[3] * A[0];
										localHessian[7] = A[3] * A[1];
										localHessian[8] = A[3] * A[2];
										localHessian[9] = A[3] * A[3];
										localHessian[10] = A[4] * A[0];
										localHessian[11] = A[4] * A[1];
										localHessian[12] = A[4] * A[2];
										localHessian[13] = A[4] * A[3];
										localHessian[14] = A[4] * A[4];
										localHessian[15] = A[5] * A[0];
										localHessian[16] = A[5] * A[1];
										localHessian[17] = A[5] * A[2];
										localHessian[18] = A[5] * A[3];
										localHessian[19] = A[5] * A[4];
										localHessian[20] = A[5] * A[5];
									}
						#endif




#if KNOB4_USE_ND_RANGE == 1

								} // if corr3Dnormal is valid
							} // if dist <= distThresh
						} // if curr3Dpoint is valid
					} // if tmp2Dpoint is valid
				} // if tmp3Dpoint_reproj.z > 0.0f
			} // if d != 0


			write_pipe(valid_pipe, &isValid);
			mem_fence(CLK_CHANNEL_MEM_FENCE);

			write_pipe(sumF_pipe, &localF);
			mem_fence(CLK_CHANNEL_MEM_FENCE);

			write_pipe(nabla_pipe0, &localNabla[0]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			write_pipe(nabla_pipe1, &localNabla[1]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			write_pipe(nabla_pipe2, &localNabla[2]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
		#if KNOB4_SHORT_ITERATION_BRANCH != 0
			if(!shortIteration)
		#endif
			{
				write_pipe(nabla_pipe3, &localNabla[3]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(nabla_pipe4, &localNabla[4]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(nabla_pipe5, &localNabla[5]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
			}

			write_pipe(hessian_pipe0, &localHessian[0]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			write_pipe(hessian_pipe1, &localHessian[1]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			write_pipe(hessian_pipe2, &localHessian[2]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			write_pipe(hessian_pipe3, &localHessian[3]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			write_pipe(hessian_pipe4, &localHessian[4]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			write_pipe(hessian_pipe5, &localHessian[5]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
		#if KNOB4_SHORT_ITERATION_BRANCH != 0
			if(!shortIteration)
		#endif
			{
				write_pipe(hessian_pipe6, &localHessian[6]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe7, &localHessian[7]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe8, &localHessian[8]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe9, &localHessian[9]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe10, &localHessian[10]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe11, &localHessian[11]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe12, &localHessian[12]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe13, &localHessian[13]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe14, &localHessian[14]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe15, &localHessian[15]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe16, &localHessian[16]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe17, &localHessian[17]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe18, &localHessian[18]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe19, &localHessian[19]);
				mem_fence(CLK_CHANNEL_MEM_FENCE);
				write_pipe(hessian_pipe20, &localHessian[20]);
			}
		}
#else

									// --------------------
									// Count valid points
									// --------------------
									noValidPoints_local++;

									// --------------------
									// Accumulate value
									// --------------------
						#if KNOB4_SUM_NABLA_TYPE != 2 && KNOB4_SUM_HESSIAN_TYPE != 2
									sumF_local += localF;
						#endif


									// --------------------
									// Accumulate Nabla
									// --------------------
						#if KNOB4_SUM_NABLA_TYPE == 0
									for (int i = 0; i < noPara; i++)   sumNabla_local[i]   += localNabla[i];
						#elif KNOB4_SUM_NABLA_TYPE == 1
									sumNabla_local[0]   += localNabla[0];
									sumNabla_local[1]   += localNabla[1];
									sumNabla_local[2]   += localNabla[2];
						#if KNOB4_SHORT_ITERATION_BRANCH != 0
									if(!shortIteration)
						#endif
									{
										sumNabla_local[3]   += localNabla[3];
										sumNabla_local[4]   += localNabla[4];
										sumNabla_local[5]   += localNabla[5];
									}
						#elif KNOB4_SUM_NABLA_TYPE == 2
									float8 nabla_buffer_cur;
									nabla_buffer_cur.s0 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s0 + localNabla[0];
									nabla_buffer_cur.s1 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s1 + localNabla[1];
									nabla_buffer_cur.s2 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s2 + localNabla[2];
						#if KNOB4_SHORT_ITERATION_BRANCH != 0
									if(!shortIteration)
						#endif
									{
										nabla_buffer_cur.s3 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s3 + localNabla[3];
										nabla_buffer_cur.s4 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s4 + localNabla[4];
										nabla_buffer_cur.s5 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s5 + localNabla[5];
									}

									// This one holds sumF_local
									nabla_buffer_cur.s6 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s6 + localF;

									for(unsigned int i = KNOB4_SHIFT_REG_SIZE-1; i > 0; i--)
									{
										nabla_shiftreg[i] = nabla_shiftreg[i-1];
									}
									nabla_shiftreg[0] = nabla_buffer_cur;
						#endif


									// --------------------
									// Accumulate Hessian
									// --------------------
						#if KNOB4_SUM_HESSIAN_TYPE == 0
									for (int i = 0; i < noParaSQ; i++) sumHessian_local[i] += localHessian[i];
						#elif KNOB4_SUM_HESSIAN_TYPE == 1
									sumHessian_local[0]  += localHessian[0];
									sumHessian_local[1]  += localHessian[1];
									sumHessian_local[2]  += localHessian[2];
									sumHessian_local[3]  += localHessian[3];
									sumHessian_local[4]  += localHessian[4];
									sumHessian_local[5]  += localHessian[5];
						#if KNOB4_SHORT_ITERATION_BRANCH != 0
									if(!shortIteration)
						#endif
									{
										sumHessian_local[6]  += localHessian[6];
										sumHessian_local[7]  += localHessian[7];
										sumHessian_local[8]  += localHessian[8];
										sumHessian_local[9]  += localHessian[9];
										sumHessian_local[10] += localHessian[10];
										sumHessian_local[11] += localHessian[11];
										sumHessian_local[12] += localHessian[12];
										sumHessian_local[13] += localHessian[13];
										sumHessian_local[14] += localHessian[14];
										sumHessian_local[15] += localHessian[15];
										sumHessian_local[16] += localHessian[16];
										sumHessian_local[17] += localHessian[17];
										sumHessian_local[18] += localHessian[18];
										sumHessian_local[19] += localHessian[19];
										sumHessian_local[20] += localHessian[20];
									}
						#elif KNOB4_SUM_HESSIAN_TYPE == 2
									float16 hessian_buffer_cur1;
									hessian_buffer_cur1.s0 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s0 + localHessian[0];
									hessian_buffer_cur1.s1 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s1 + localHessian[1];
									hessian_buffer_cur1.s2 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s2 + localHessian[2];
									hessian_buffer_cur1.s3 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s3 + localHessian[3];
									hessian_buffer_cur1.s4 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s4 + localHessian[4];
									hessian_buffer_cur1.s5 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s5 + localHessian[5];
						#if KNOB4_SHORT_ITERATION_BRANCH != 0
									if(shortIteration)
									{
						#if KNOB4_SUM_NABLA_TYPE != 2
										// This one holds sumF_local
										hessian_buffer_cur1.s6 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s6 + localF;
						#endif
									}
									else
						#endif
									{
										hessian_buffer_cur1.s6 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s6 + localHessian[6];
										hessian_buffer_cur1.s7 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s7 + localHessian[7];
										hessian_buffer_cur1.s8 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s8 + localHessian[8];
										hessian_buffer_cur1.s9 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s9 + localHessian[9];
										hessian_buffer_cur1.sa = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].sa + localHessian[10];
										hessian_buffer_cur1.sb = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].sb + localHessian[11];
										hessian_buffer_cur1.sc = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].sc + localHessian[12];
										hessian_buffer_cur1.sd = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].sd + localHessian[13];
										hessian_buffer_cur1.se = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].se + localHessian[14];
										hessian_buffer_cur1.sf = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].sf + localHessian[15];
									}

									for(unsigned int i = KNOB4_SHIFT_REG_SIZE-1; i > 0; i--)
									{
										hessian_shiftreg1[i] = hessian_shiftreg1[i-1];
									}
									hessian_shiftreg1[0] = hessian_buffer_cur1;

						#if KNOB4_SHORT_ITERATION_BRANCH != 0
									if(!shortIteration)
						#endif
									{
										float8  hessian_buffer_cur2;
										hessian_buffer_cur2.s0  = hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE-1].s0  + localHessian[16];
										hessian_buffer_cur2.s1  = hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE-1].s1  + localHessian[17];
										hessian_buffer_cur2.s2  = hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE-1].s2  + localHessian[18];
										hessian_buffer_cur2.s3  = hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE-1].s3  + localHessian[19];
										hessian_buffer_cur2.s4  = hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE-1].s4  + localHessian[20];

						#if KNOB4_SUM_NABLA_TYPE != 2
										// This one holds sumF_local
										hessian_buffer_cur2.s5 = hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE-1].s5 + localF;
						#endif

										for(unsigned int i = KNOB4_SHIFT_REG_SIZE-1; i > 0; i--)
										{
											hessian_shiftreg2[i] = hessian_shiftreg2[i-1];
										}
										hessian_shiftreg2[0] = hessian_buffer_cur2;
									}
						#endif
								} // if corr3Dnormal is valid
							} // if dist <= distThresh
						} // if curr3Dpoint is valid
					} // if tmp2Dpoint is valid
				} // if tmp3Dpoint_reproj.z > 0.0f
			}// if d != 0

		}// x
	// y


	// Store Nabla
#if KNOB4_SUM_NABLA_TYPE == 2
	for(unsigned int i = 0; i < KNOB4_SHIFT_REG_SIZE; i++)
	{
		nabla_buffer += nabla_shiftreg[i];
	}
	sumNabla[0] = nabla_buffer.s0;
	sumNabla[1] = nabla_buffer.s1;
	sumNabla[2] = nabla_buffer.s2;
#if KNOB4_SHORT_ITERATION_BRANCH != 0
	if(!shortIteration)
#endif
	{
		sumNabla[3] = nabla_buffer.s3;
		sumNabla[4] = nabla_buffer.s4;
		sumNabla[5] = nabla_buffer.s5;
	}
#else
	for (int i = 0; i < noPara; i++)   sumNabla[i]   = sumNabla_local[i];
#endif

	// Store Hessian
#if KNOB4_SUM_HESSIAN_TYPE == 2
	for(unsigned int i = 0; i < KNOB4_SHIFT_REG_SIZE; i++)
	{
		hessian_buffer1 += hessian_shiftreg1[i];
	}
	sumHessian[0]  = hessian_buffer1.s0;
	sumHessian[1]  = hessian_buffer1.s1;
	sumHessian[2]  = hessian_buffer1.s2;
	sumHessian[3]  = hessian_buffer1.s3;
	sumHessian[4]  = hessian_buffer1.s4;
	sumHessian[5]  = hessian_buffer1.s5;
#if KNOB4_SHORT_ITERATION_BRANCH != 0
	if(!shortIteration)
#endif
	{
		sumHessian[6]  = hessian_buffer1.s6;
		sumHessian[7]  = hessian_buffer1.s7;
		sumHessian[8]  = hessian_buffer1.s8;
		sumHessian[9]  = hessian_buffer1.s9;
		sumHessian[10] = hessian_buffer1.sa;
		sumHessian[11] = hessian_buffer1.sb;
		sumHessian[12] = hessian_buffer1.sc;
		sumHessian[13] = hessian_buffer1.sd;
		sumHessian[14] = hessian_buffer1.se;
		sumHessian[15] = hessian_buffer1.sf;

		for(unsigned int i = 0; i < KNOB4_SHIFT_REG_SIZE; i++)
		{
			hessian_buffer2 += hessian_shiftreg2[i];
		}

		sumHessian[16] = hessian_buffer2.s0;
		sumHessian[17] = hessian_buffer2.s1;
		sumHessian[18] = hessian_buffer2.s2;
		sumHessian[19] = hessian_buffer2.s3;
		sumHessian[20] = hessian_buffer2.s4;
	}
#else
	for (int i = 0; i < noParaSQ; i++) sumHessian[i] = sumHessian_local[i];
#endif

	// Store num valid points
	*noValidPoints = noValidPoints_local;

	// Store F
#if KNOB4_SUM_NABLA_TYPE == 2
	*sumF = nabla_buffer.s6;
#elif KNOB4_SUM_HESSIAN_TYPE == 2
#if KNOB4_SHORT_ITERATION_BRANCH != 0
	if(shortIteration)
	{
		*sumF = hessian_buffer1.s6;
	}
	else
#endif
	{
		*sumF = hessian_buffer2.s5;
	}
#else
	*sumF = sumF_local;
#endif

#endif // Use ND range

}

#if KNOB4_USE_ND_RANGE == 1

__attribute__((max_global_work_dim(0)))
kernel void icp_accumulate(
		global float* restrict sumHessian,
		global float* restrict sumNabla,
		global int*   restrict noValidPoints,
		global float* restrict sumF,
		int numElements,
		uchar shortIteration,
		read_only pipe uchar __attribute__((blocking))
		                     __attribute__((depth(KNOB4_PIPE_DEPTH))) valid_pipe,
		read_only pipe float __attribute__((blocking))
		                     __attribute__((depth(KNOB4_PIPE_DEPTH))) sumF_pipe,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) nabla_pipe0,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) nabla_pipe1,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) nabla_pipe2,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) nabla_pipe3,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) nabla_pipe4,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) nabla_pipe5,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe0,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe1,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe2,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe3,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe4,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe5,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe6,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe7,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe8,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe9,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe10,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe11,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe12,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe13,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe14,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe15,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe16,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe17,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe18,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe19,
		read_only pipe float __attribute__((blocking))
							 __attribute__((depth(KNOB4_PIPE_DEPTH))) hessian_pipe20
)
{
#if KNOB4_SUM_NABLA_TYPE != 2 || KNOB4_UNROLL_NABLA == 0
#if KNOB4_SHORT_ITERATION_BRANCH != 0
	const int noPara = shortIteration ? 3 : 6;
#else
	const int noPara = 6;
#endif
#endif

#if KNOB4_SUM_HESSIAN_TYPE != 2
#if KNOB4_SHORT_ITERATION_BRANCH != 0
	const int noParaSQ = shortIteration ? SHORT_HESSIAN_SIZE : HESSIAN_SIZE;
#else
	const int noParaSQ = HESSIAN_SIZE;
#endif
#endif


	int noValidPoints_local = 0;

#if KNOB4_SUM_NABLA_TYPE != 2 && KNOB4_SUM_HESSIAN_TYPE != 2
	float sumF_local        = 0.f;
#endif


	// Initialize Nabla buffer
#if KNOB4_SUM_NABLA_TYPE == 2
	float8 nabla_buffer = (float8)(0,0,0,0,0,0,0,0);
	float8 nabla_shiftreg[KNOB4_SHIFT_REG_SIZE];
	for(unsigned int i = 0; i < KNOB4_SHIFT_REG_SIZE; i++)
	{
		nabla_shiftreg[i] = nabla_buffer;
	}
#else
	float sumNabla_local[6];
	for (int i = 0; i < 6; i++) sumNabla_local[i] = 0.0f;
#endif


	// Initialize Hessian buffer
#if KNOB4_SUM_HESSIAN_TYPE == 2
	float16 hessian_buffer1 = (float16)(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
	float8  hessian_buffer2 = (float8) (0,0,0,0,0,0,0,0);
	float16 hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE];
	float8  hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE];
	for(unsigned int i = 0; i < KNOB4_SHIFT_REG_SIZE; i++)
	{
		hessian_shiftreg1[i] = hessian_buffer1;
		hessian_shiftreg2[i] = hessian_buffer2;
	}
#else
	float sumHessian_local[HESSIAN_SIZE];
	for (int i = 0; i < HESSIAN_SIZE; i++) sumHessian_local[i] = 0.0f;
#endif


	for(int num = 0; num < numElements; num++)
	{
		uchar isValid;
		float localF;
		float localNabla[6];
		float localHessian[HESSIAN_SIZE];

		read_pipe(valid_pipe, &isValid);
		mem_fence(CLK_CHANNEL_MEM_FENCE);

		read_pipe(sumF_pipe, &localF);
		mem_fence(CLK_CHANNEL_MEM_FENCE);

		read_pipe(nabla_pipe0, &localNabla[0]);
		mem_fence(CLK_CHANNEL_MEM_FENCE);
		read_pipe(nabla_pipe1, &localNabla[1]);
		mem_fence(CLK_CHANNEL_MEM_FENCE);
		read_pipe(nabla_pipe2, &localNabla[2]);
		mem_fence(CLK_CHANNEL_MEM_FENCE);
#if KNOB4_SHORT_ITERATION_BRANCH != 0
		if(!shortIteration)
#endif
		{
			read_pipe(nabla_pipe3, &localNabla[3]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(nabla_pipe4, &localNabla[4]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(nabla_pipe5, &localNabla[5]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
		}

		read_pipe(hessian_pipe0, &localHessian[0]);
		mem_fence(CLK_CHANNEL_MEM_FENCE);
		read_pipe(hessian_pipe1, &localHessian[1]);
		mem_fence(CLK_CHANNEL_MEM_FENCE);
		read_pipe(hessian_pipe2, &localHessian[2]);
		mem_fence(CLK_CHANNEL_MEM_FENCE);
		read_pipe(hessian_pipe3, &localHessian[3]);
		mem_fence(CLK_CHANNEL_MEM_FENCE);
		read_pipe(hessian_pipe4, &localHessian[4]);
		mem_fence(CLK_CHANNEL_MEM_FENCE);
		read_pipe(hessian_pipe5, &localHessian[5]);
		mem_fence(CLK_CHANNEL_MEM_FENCE);
#if KNOB4_SHORT_ITERATION_BRANCH != 0
		if(!shortIteration)
#endif
		{
			read_pipe(hessian_pipe6, &localHessian[6]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe7, &localHessian[7]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe8, &localHessian[8]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe9, &localHessian[9]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe10, &localHessian[10]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe11, &localHessian[11]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe12, &localHessian[12]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe13, &localHessian[13]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe14, &localHessian[14]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe15, &localHessian[15]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe16, &localHessian[16]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe17, &localHessian[17]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe18, &localHessian[18]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe19, &localHessian[19]);
			mem_fence(CLK_CHANNEL_MEM_FENCE);
			read_pipe(hessian_pipe20, &localHessian[20]);
		}



		// --------------------
		// Count valid points
		// --------------------
		noValidPoints_local += isValid;

		// --------------------
		// Accumulate value
		// --------------------
#if KNOB4_SUM_NABLA_TYPE != 2 && KNOB4_SUM_HESSIAN_TYPE != 2
		sumF_local += localF;
#endif


		// --------------------
		// Accumulate Nabla
		// --------------------
#if KNOB4_SUM_NABLA_TYPE == 0
		for (int i = 0; i < noPara; i++)   sumNabla_local[i]   += localNabla[i];
#elif KNOB4_SUM_NABLA_TYPE == 1
		sumNabla_local[0]   += localNabla[0];
		sumNabla_local[1]   += localNabla[1];
		sumNabla_local[2]   += localNabla[2];
#if KNOB4_SHORT_ITERATION_BRANCH != 0
		if(!shortIteration)
#endif
		{
			sumNabla_local[3]   += localNabla[3];
			sumNabla_local[4]   += localNabla[4];
			sumNabla_local[5]   += localNabla[5];
		}
#elif KNOB4_SUM_NABLA_TYPE == 2
		float8 nabla_buffer_cur;
		nabla_buffer_cur.s0 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s0 + localNabla[0];
		nabla_buffer_cur.s1 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s1 + localNabla[1];
		nabla_buffer_cur.s2 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s2 + localNabla[2];
#if KNOB4_SHORT_ITERATION_BRANCH != 0
		if(!shortIteration)
#endif
		{
			nabla_buffer_cur.s3 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s3 + localNabla[3];
			nabla_buffer_cur.s4 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s4 + localNabla[4];
			nabla_buffer_cur.s5 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s5 + localNabla[5];
		}

		// This one holds sumF_local
		nabla_buffer_cur.s6 = nabla_shiftreg[KNOB4_SHIFT_REG_SIZE-1].s6 + localF;

		for(unsigned int i = KNOB4_SHIFT_REG_SIZE-1; i > 0; i--)
		{
			nabla_shiftreg[i] = nabla_shiftreg[i-1];
		}
		nabla_shiftreg[0] = nabla_buffer_cur;
#endif



		// --------------------
		// Accumulate Hessian
		// --------------------
#if KNOB4_SUM_HESSIAN_TYPE == 0
		for (int i = 0; i < noParaSQ; i++) sumHessian_local[i] += localHessian[i];
#elif KNOB4_SUM_HESSIAN_TYPE == 1
		sumHessian_local[0]  += localHessian[0];
		sumHessian_local[1]  += localHessian[1];
		sumHessian_local[2]  += localHessian[2];
		sumHessian_local[3]  += localHessian[3];
		sumHessian_local[4]  += localHessian[4];
		sumHessian_local[5]  += localHessian[5];
#if KNOB4_SHORT_ITERATION_BRANCH != 0
		if(!shortIteration)
#endif
		{
			sumHessian_local[6]  += localHessian[6];
			sumHessian_local[7]  += localHessian[7];
			sumHessian_local[8]  += localHessian[8];
			sumHessian_local[9]  += localHessian[9];
			sumHessian_local[10] += localHessian[10];
			sumHessian_local[11] += localHessian[11];
			sumHessian_local[12] += localHessian[12];
			sumHessian_local[13] += localHessian[13];
			sumHessian_local[14] += localHessian[14];
			sumHessian_local[15] += localHessian[15];
			sumHessian_local[16] += localHessian[16];
			sumHessian_local[17] += localHessian[17];
			sumHessian_local[18] += localHessian[18];
			sumHessian_local[19] += localHessian[19];
			sumHessian_local[20] += localHessian[20];
		}
#elif KNOB4_SUM_HESSIAN_TYPE == 2
		float16 hessian_buffer_cur1;
		hessian_buffer_cur1.s0 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s0 + localHessian[0];
		hessian_buffer_cur1.s1 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s1 + localHessian[1];
		hessian_buffer_cur1.s2 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s2 + localHessian[2];
		hessian_buffer_cur1.s3 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s3 + localHessian[3];
		hessian_buffer_cur1.s4 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s4 + localHessian[4];
		hessian_buffer_cur1.s5 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s5 + localHessian[5];
#if KNOB4_SHORT_ITERATION_BRANCH != 0
		if(shortIteration)
		{
#if KNOB4_SUM_NABLA_TYPE != 2
			// This one holds sumF_local
			hessian_buffer_cur1.s6 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s6 + localF;
#endif
		}
		else
#endif
		{
			hessian_buffer_cur1.s6 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s6 + localHessian[6];
			hessian_buffer_cur1.s7 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s7 + localHessian[7];
			hessian_buffer_cur1.s8 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s8 + localHessian[8];
			hessian_buffer_cur1.s9 = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].s9 + localHessian[9];
			hessian_buffer_cur1.sa = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].sa + localHessian[10];
			hessian_buffer_cur1.sb = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].sb + localHessian[11];
			hessian_buffer_cur1.sc = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].sc + localHessian[12];
			hessian_buffer_cur1.sd = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].sd + localHessian[13];
			hessian_buffer_cur1.se = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].se + localHessian[14];
			hessian_buffer_cur1.sf = hessian_shiftreg1[KNOB4_SHIFT_REG_SIZE-1].sf + localHessian[15];
		}

		for(unsigned int i = KNOB4_SHIFT_REG_SIZE-1; i > 0; i--)
		{
			hessian_shiftreg1[i] = hessian_shiftreg1[i-1];
		}
		hessian_shiftreg1[0] = hessian_buffer_cur1;

#if KNOB4_SHORT_ITERATION_BRANCH != 0
		if(!shortIteration)
#endif
		{
			float8  hessian_buffer_cur2;
			hessian_buffer_cur2.s0  = hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE-1].s0  + localHessian[16];
			hessian_buffer_cur2.s1  = hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE-1].s1  + localHessian[17];
			hessian_buffer_cur2.s2  = hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE-1].s2  + localHessian[18];
			hessian_buffer_cur2.s3  = hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE-1].s3  + localHessian[19];
			hessian_buffer_cur2.s4  = hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE-1].s4  + localHessian[20];

#if KNOB4_SUM_NABLA_TYPE != 2
			// This one holds sumF_local
			hessian_buffer_cur2.s5 = hessian_shiftreg2[KNOB4_SHIFT_REG_SIZE-1].s5 + localF;
#endif

			for(unsigned int i = KNOB4_SHIFT_REG_SIZE-1; i > 0; i--)
			{
				hessian_shiftreg2[i] = hessian_shiftreg2[i-1];
			}
			hessian_shiftreg2[0] = hessian_buffer_cur2;
		}
#endif
	} // for each element



	// Store Nabla
#if KNOB4_SUM_NABLA_TYPE == 2
	for(unsigned int i = 0; i < KNOB4_SHIFT_REG_SIZE; i++)
	{
		nabla_buffer += nabla_shiftreg[i];
	}
	sumNabla[0] = nabla_buffer.s0;
	sumNabla[1] = nabla_buffer.s1;
	sumNabla[2] = nabla_buffer.s2;
#if KNOB4_SHORT_ITERATION_BRANCH != 0
	if(!shortIteration)
#endif
	{
		sumNabla[3] = nabla_buffer.s3;
		sumNabla[4] = nabla_buffer.s4;
		sumNabla[5] = nabla_buffer.s5;
	}
#else
	for (int i = 0; i < noPara; i++)   sumNabla[i]   = sumNabla_local[i];
#endif

	// Store Hessian
#if KNOB4_SUM_HESSIAN_TYPE == 2
	for(unsigned int i = 0; i < KNOB4_SHIFT_REG_SIZE; i++)
	{
		hessian_buffer1 += hessian_shiftreg1[i];
	}
	sumHessian[0]  = hessian_buffer1.s0;
	sumHessian[1]  = hessian_buffer1.s1;
	sumHessian[2]  = hessian_buffer1.s2;
	sumHessian[3]  = hessian_buffer1.s3;
	sumHessian[4]  = hessian_buffer1.s4;
	sumHessian[5]  = hessian_buffer1.s5;
#if KNOB4_SHORT_ITERATION_BRANCH != 0
	if(!shortIteration)
#endif
	{
		sumHessian[6]  = hessian_buffer1.s6;
		sumHessian[7]  = hessian_buffer1.s7;
		sumHessian[8]  = hessian_buffer1.s8;
		sumHessian[9]  = hessian_buffer1.s9;
		sumHessian[10] = hessian_buffer1.sa;
		sumHessian[11] = hessian_buffer1.sb;
		sumHessian[12] = hessian_buffer1.sc;
		sumHessian[13] = hessian_buffer1.sd;
		sumHessian[14] = hessian_buffer1.se;
		sumHessian[15] = hessian_buffer1.sf;

		for(unsigned int i = 0; i < KNOB4_SHIFT_REG_SIZE; i++)
		{
			hessian_buffer2 += hessian_shiftreg2[i];
		}

		sumHessian[16] = hessian_buffer2.s0;
		sumHessian[17] = hessian_buffer2.s1;
		sumHessian[18] = hessian_buffer2.s2;
		sumHessian[19] = hessian_buffer2.s3;
		sumHessian[20] = hessian_buffer2.s4;
	}
#else
	for (int i = 0; i < noParaSQ; i++) sumHessian[i] = sumHessian_local[i];
#endif

	// Store num valid points
	*noValidPoints = noValidPoints_local;

	// Store F
#if KNOB4_SUM_NABLA_TYPE == 2
	*sumF = nabla_buffer.s6;
#elif KNOB4_SUM_HESSIAN_TYPE == 2
#if KNOB4_SHORT_ITERATION_BRANCH != 0
	if(shortIteration)
	{
		*sumF = hessian_buffer1.s6;
	}
	else
#endif
	{
		*sumF = hessian_buffer2.s5;
	}
#else
	*sumF = sumF_local;
#endif

}

#endif // KNOB4_USE_ND_RANGE


#endif // ENABLE_KERNEL_ICP












