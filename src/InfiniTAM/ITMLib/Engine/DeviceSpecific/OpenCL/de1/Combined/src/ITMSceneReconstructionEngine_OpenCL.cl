#include "ITMSceneReconstructionEngine_OpenCL_knobs.h"

// Matrix-vector multiply (4x4)
#define matVecMul(m, vec) ((float4) (dot((m).data0, (vec)), dot((m).data1, (vec)), dot((m).data2, (vec)), dot((m).data3, (vec))))
#define matVecMul3(m, vec) ((float3) (dot((m).data0, (vec)), dot((m).data1, (vec)), dot((m).data2, (vec))))



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

#ifdef KERNEL_INTEGRATEINTOSCENE_DEPTH_S

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


#endif // KERNEL_INTEGRATEINTOSCENE_DEPTH_S




// ***************************************************************************

#ifdef KERNEL_RAYCAST_DEPTH_S

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

#endif // KERNEL_RAYCAST_DEPTH_S



// ***************************************************************************

#ifdef KERNEL_INTEGRATEINTOSCENE_DEPTH_S_COMBINED

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


#endif // KERNEL_INTEGRATEINTOSCENE_DEPTH_S_COMBINED










