//
// Created by qkgautier on 8/5/16.
//

#pragma once

#include "../../ITMExportEngine.h"

namespace ITMLib
{
    namespace Engine
    {
        template<class TVoxel, class TIndex>
        class ITMExportEngine_CUDA : public ITMExportEngine <TVoxel, TIndex>
        {
        public:

            ITMExportEngine_CUDA(Objects::ITMScene<TVoxel, TIndex>* scene = 0,
					Objects::ITMTrackingState* tracking = 0):
				ITMExportEngine<TVoxel, TIndex>(scene, tracking) {}

            virtual ~ITMExportEngine_CUDA(){}

            virtual bool ExportTSDFToPcd(const char* filename);


        private:

            int countVoxelsInBlock(const TVoxel* voxelBlocks, int block_ptr);

            void getVoxelsFromPtrList(int noBlocks,
                                      const TVoxel* voxelBlocks_device,
                                      int* blockPtr_host,
                                      TVoxel* voxels_host);

            void exportVoxelsInBlock(std::ofstream& file, const TVoxel* voxelBlocks,
                                     int block_ptr, Vector3s blockPos);

            bool ExportTSDFToPcd_hashIndex(Objects::ITMScene<TVoxel, Objects::ITMVoxelBlockHash>* scene,
                                           const char *filename);
        };
    }
}
