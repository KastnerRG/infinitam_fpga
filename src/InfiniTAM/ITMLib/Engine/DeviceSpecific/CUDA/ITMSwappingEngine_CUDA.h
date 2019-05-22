// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMSwappingEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMSwappingEngine_CUDA : public ITMSwappingEngine < TVoxel, TIndex >
		{
		public:
			void IntegrateGlobalIntoLocal(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
			void SaveToGlobalMemory(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
		};

		template<class TVoxel>
		class ITMSwappingEngine_CUDA<TVoxel, ITMVoxelBlockHash> : public ITMSwappingEngine < TVoxel, ITMVoxelBlockHash >
		{
		private:
			int *noNeededEntries_device, *noAllocatedVoxelEntries_device;
			int LoadFromGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

		public:
			void IntegrateGlobalIntoLocal(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);
			void SaveToGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);

			ITMSwappingEngine_CUDA(void);
			~ITMSwappingEngine_CUDA(void);
		};
	}
}
