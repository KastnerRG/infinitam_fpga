
// *****************************************************************
// This is an example Depth Fusion design where all the knobs have been resolved.
// *****************************************************************

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
	float4 data0;
	float4 data1;
	float4 data2;
	float4 data3;

} Mat44_cl;


inline float SDF_shortToFloat(short x) { return (float)(x) / (float)32767; }
inline short SDF_floatToShort(float x) { return (short)((x) * (float)32767); }



// ***************************************************************************
// Depth Fusion
// ***************************************************************************



__attribute__ ((reqd_work_group_size(8, 8, 8)))
__attribute__ ((num_simd_work_items(1)))
__attribute__ ((num_compute_units(2)))

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
	// *** Loop entryId
	//

	const int entryId = get_group_id(0);

	{
		const ITMHashEntry currentHashEntry = hashTable[visibleEntryIds[entryId]];


		int3 globalPos;
		globalPos.x = currentHashEntry.pos.x;
		globalPos.y = currentHashEntry.pos.y;
		globalPos.z = currentHashEntry.pos.z;
		globalPos *= 8;

		global ITMVoxel_s *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (512)]);


		// *** Loop x,y,z


		const int x = get_local_id(0);
		const int y = get_local_id(1);
		const int z = get_local_id(2);

		int locId = x + y * 8 + z * 8 * 8;
		{
			float4 pt_model;
			pt_model.x = (float)(globalPos.x + x) * voxelSize;
			pt_model.y = (float)(globalPos.y + y) * voxelSize;
			pt_model.z = (float)(globalPos.z + z) * voxelSize;
			pt_model.w = 1.0f;


			// Project point into image
			float4 pt_camera = ((float4) (dot((*M_d).data0, (pt_model)), dot((*M_d).data1, (pt_model)), dot((*M_d).data2, (pt_model)), dot((*M_d).data3, (pt_model))));
			if (pt_camera.z <= 0) { return; }

			float2 pt_image;
			pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
			pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
			if ((pt_image.x < 1) || (pt_image.x > (depthImgSize.x) - 2) || (pt_image.y < 1) || (pt_image.y > (depthImgSize.y) - 2)) return;

			// Get measured depth from image
			float depth_measure = depth[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * (depthImgSize.x)];
			if (depth_measure <= 0.0) return;

			// Check whether voxel needs updating
			float eta = depth_measure - pt_camera.z;
			if (eta < -mu) return;

			// Get voxel here and check for max weight
			ITMVoxel_s voxel = localVoxelBlock[locId];

			if (stopIntegratingAtMaxW) if (voxel.w_depth == maxW) return;


			// Compute updated SDF value and reliability
			float oldF = SDF_shortToFloat(voxel.sdf);
			int oldW = voxel.w_depth;

			float newF = min(1.0f, eta / mu);
			int newW = 1;

			newF = oldW * oldF + newW * newF;
			newW = oldW + newW;
			newF /= newW;
			newW = min(newW, maxW);

			// Write back
			localVoxelBlock[locId].sdf = SDF_floatToShort(newF);
			localVoxelBlock[locId].w_depth = newW;


		} // for x,y,z
	} // for entryId
}





