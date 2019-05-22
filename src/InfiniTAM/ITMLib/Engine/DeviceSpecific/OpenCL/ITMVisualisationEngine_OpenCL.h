// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMVisualisationEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMVisualisationEngine_OpenCL : public ITMVisualisationEngine < TVoxel, TIndex >
		{
		public:
			explicit ITMVisualisationEngine_OpenCL(ITMScene<TVoxel, TIndex> *scene) : ITMVisualisationEngine<TVoxel, TIndex>(scene) { }
			~ITMVisualisationEngine_OpenCL(void) { }

			void FindVisibleBlocks(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void CreateExpectedDepths(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void RenderImage(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, 
				ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE) const;
			void FindSurface(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
			void CreatePointCloud(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
			void CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
			void ForwardRender(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;

			ITMRenderState* CreateRenderState(const Vector2i & imgSize) const;
		};

		template<class TVoxel>
		class ITMVisualisationEngine_OpenCL<TVoxel, ITMVoxelBlockHash> : public ITMVisualisationEngine < TVoxel, ITMVoxelBlockHash >
		{
		public:
			explicit ITMVisualisationEngine_OpenCL(ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
			: ITMVisualisationEngine<TVoxel, ITMVoxelBlockHash>(scene), invM_device_(0)
			  {
#ifdef OPENCL_DEBUG
				pointsRay_device_  = 0;
				voxelData_device_  = 0;
				voxelIndex_device_ = 0;
				minmaximg_device_  = 0;
#endif
			  }
			~ITMVisualisationEngine_OpenCL(void);

			void FindVisibleBlocks(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void CreateExpectedDepths(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void RenderImage(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, 
				ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE) const;
			void FindSurface(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
			void CreatePointCloud(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
			void CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
			void ForwardRender(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;

			ITMRenderState_VH* CreateRenderState(const Vector2i & imgSize) const;

		protected:
			cl_mem invM_device_;

#ifdef OPENCL_DEBUG
			cl_mem pointsRay_device_;
			cl_mem voxelData_device_;
			cl_mem voxelIndex_device_;
			cl_mem minmaximg_device_;
#endif
		};
	}
}
