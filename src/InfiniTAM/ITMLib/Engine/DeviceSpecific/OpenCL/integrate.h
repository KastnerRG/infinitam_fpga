
//#define SDF_BLOCK_SIZE 8
//#define SDF_BLOCK_SIZE3 512

namespace reconstruct_hls
{

typedef unsigned char uchar;

typedef struct
{
	union{
		struct { float r, g, b, a; };
		struct { float x, y, z, w; };
		float at[4];
	};
} float4;

typedef struct
{
	union{
		struct { float x, y; };
		float at[2];
	};
} float2;

typedef struct
{
	union{
		struct { int x, y; };
		int at[2];
	};
} int2;

typedef struct
{
	union{
		struct { int x, y, z; };
		int at[3];
	};
} int3;

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

} Mat44;




void IntegrateIntoScene_depth_s(
		int2 depthImgSize,
		float voxelSize,
		const Mat44* M_d,
		float4 projParams_d,
		float mu,
		int maxW,
		const float* depth,
		ITMVoxel_s* localVBA,
		const ITMHashEntry* hashTable,
		const int* visibleEntryIds,
		int noVisibleEntries,
		uchar stopIntegratingAtMaxW
		);

}

