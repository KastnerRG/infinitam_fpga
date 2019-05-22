// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMSceneReconstructionEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMSceneReconstructionEngine_OpenCL : public ITMSceneReconstructionEngine < TVoxel, TIndex >
		{};

		template<class TVoxel>
		class ITMSceneReconstructionEngine_OpenCL<TVoxel, ITMVoxelBlockHash> : public ITMSceneReconstructionEngine < TVoxel, ITMVoxelBlockHash >
		{
		protected:
			ORUtils::MemoryBlock<unsigned char> *entriesAllocType;
			ORUtils::MemoryBlock<Vector4s> *blockCoords;

			ORUtils::MemoryBlock<Matrix4f> *M_d_buffer;
			ORUtils::MemoryBlock<Matrix4f> *invM_d_buffer;
			ORUtils::Image<Vector4f> *pointsRay_buffer;
			ORUtils::Image<short> *projectedSdf_s_buffer;

#ifdef OPENCL_DEBUG
			cl_mem visibleEntryIds_device_;
			cl_mem depth_device_;
			cl_mem localVBA_device_;
			cl_mem hashTable_device_;
			cl_mem M_d_device_;
			cl_mem invM_d_device_;
			cl_mem pointsRay_device_;
			cl_mem projectedSdf_device_;
#endif

		public:
			void ResetScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

			void AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState, bool onlyUpdateVisibleList = false);

			void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState);

			ITMSceneReconstructionEngine_OpenCL(void);
			~ITMSceneReconstructionEngine_OpenCL(void);
		};

		template<class TVoxel>
		class ITMSceneReconstructionEngine_OpenCL<TVoxel, ITMPlainVoxelArray> : public ITMSceneReconstructionEngine < TVoxel, ITMPlainVoxelArray >
		{
		public:
			void ResetScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene);

			void AllocateSceneFromDepth(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState, bool onlyUpdateVisibleList = false);

			void IntegrateIntoScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState);

			ITMSceneReconstructionEngine_OpenCL(void);
			~ITMSceneReconstructionEngine_OpenCL(void);
		};
	}
}
