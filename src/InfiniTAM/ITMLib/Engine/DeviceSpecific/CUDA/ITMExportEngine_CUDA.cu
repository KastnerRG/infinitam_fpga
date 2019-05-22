
#include "ITMExportEngine_CUDA.h"

#include "../../../../ORUtils/CUDADefines.h"

#include <fstream>
#include <vector>

using namespace ITMLib::Engine;
using namespace ITMLib::Objects;


namespace ITMLib
{
    namespace Engine
    {
        template<class TVoxel>
        __global__ void getVoxelsFromPtrList_device(int noBlocks, int voxelsPerBlock,
                                                    const int* blockPtr,
                                                    const TVoxel* voxels_in, TVoxel* voxels_out)
        {
            int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
            if (targetIdx > noBlocks - 1) return;

            for(int i = 0; i < voxelsPerBlock; i++)
            {
                voxels_out[targetIdx * voxelsPerBlock + i] =
                        voxels_in[blockPtr[targetIdx] * voxelsPerBlock + i];
            }
        }
    }
}


namespace ITMLib
{
    namespace Engine
    {
        template<class TVoxel, class TIndex>
        int ITMExportEngine_CUDA<TVoxel, TIndex>::countVoxelsInBlock(const TVoxel* voxelBlocks,
                                                                     int block_ptr)
        {
            int noPoints = 0;

            for(int z = 0; z < SDF_BLOCK_SIZE; z++)
            {
                for(int y = 0; y < SDF_BLOCK_SIZE; y++)
                {
                    for(int x = 0; x < SDF_BLOCK_SIZE; x++)
                    {
                        int point_offset = x + y*SDF_BLOCK_SIZE + z*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;

                        const TVoxel& voxel = voxelBlocks[block_ptr * SDF_BLOCK_SIZE3 + point_offset];

                        if(voxel.sdf != TVoxel::SDF_initialValue()){ noPoints++; }
                    }
                }
            }

            return noPoints;
        }


        template<class TVoxel, class TIndex>
        void ITMExportEngine_CUDA<TVoxel, TIndex>::
        getVoxelsFromPtrList(int noBlocks,
                             const TVoxel* voxelBlocks_device,
                             int* blockPtr_host,
                             TVoxel* voxels_host)
        {
            TVoxel* voxels_device;
            int* blockPtr_device;

            ITMSafeCall(cudaMalloc((void**)&voxels_device, noBlocks*SDF_BLOCK_SIZE3*sizeof(TVoxel)));
            ITMSafeCall(cudaMalloc((void**)&blockPtr_device, noBlocks*sizeof(int)));

            ITMSafeCall(cudaMemcpy(blockPtr_device, blockPtr_host, noBlocks*sizeof(int), cudaMemcpyHostToDevice));

            dim3 cudaBlockSizeAL(256, 1);
            dim3 gridSizeAL((int)ceil((float)noBlocks / (float)cudaBlockSizeAL.x));

            getVoxelsFromPtrList_device<<<gridSizeAL, cudaBlockSizeAL>>>(
                    noBlocks, SDF_BLOCK_SIZE3, blockPtr_device,
                    voxelBlocks_device, voxels_device);

            ITMSafeCall(cudaMemcpy(voxels_host, voxels_device, noBlocks*SDF_BLOCK_SIZE3*sizeof(TVoxel), cudaMemcpyDeviceToHost));

            ITMSafeCall(cudaFree(voxels_device));
            ITMSafeCall(cudaFree(blockPtr_device));
        }

        template<class TVoxel, class TIndex>
        void ITMExportEngine_CUDA<TVoxel, TIndex>::exportVoxelsInBlock(std::ofstream& file,
                                                                       const TVoxel* voxelBlocks,
                                                                       int block_ptr,
                                                                       Vector3s blockPos)
        {
            Vector3f pointPos = blockPos.toFloat() * SDF_BLOCK_SIZE;

            for(int z = 0; z < SDF_BLOCK_SIZE; z++)
            {
                for(int y = 0; y < SDF_BLOCK_SIZE; y++)
                {
                    for(int x = 0; x < SDF_BLOCK_SIZE; x++)
                    {
                        int point_offset = x + y*SDF_BLOCK_SIZE + z*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
                        int voxel_ptr = block_ptr * SDF_BLOCK_SIZE3 + point_offset;
                        const TVoxel& voxel = voxelBlocks[voxel_ptr];

                        if(voxel.sdf == TVoxel::SDF_initialValue()){ continue; }

                        // Export X,Y,Z
                        Vector3f currPointPos = pointPos + Vector3f(x, y, z);
                        //file << currPointPos.x << " " << currPointPos.y << " " << currPointPos.z << " ";
                        file.write((const char*)&currPointPos, 3 * sizeof(float));


                        // Export colors
                        if(TVoxel::hasColorInformation)
                        {
                            // hack to get color information without using templates
                            // TODO use templates
                            const uchar* raw_ptr = reinterpret_cast<const uchar*>(voxelBlocks);
                            const uchar* clr_ptr = raw_ptr + (voxel_ptr+1) * sizeof(TVoxel) -
                                    sizeof(uchar) - sizeof(Vector3u) - 1;

                            unsigned int rgb = ((unsigned int)clr_ptr[0] << 16 | (unsigned int)clr_ptr[1] << 8 | (unsigned int)clr_ptr[2]);
                            float rgb_f = *reinterpret_cast<float*>(&rgb);

                            //file << rgb_f << " ";
                            file.write((const char*)&rgb_f, sizeof(float));
                        }

                        // Export SDF value
                        float tsdf = TVoxel::SDF_valueToFloat(voxel.sdf);
                        //file << tsdf << "\n";
                        file.write((const char*)&tsdf, sizeof(float));
                    }
                }
            }
        }


        template<class TVoxel, class TIndex>
        bool ITMExportEngine_CUDA<TVoxel, TIndex>::ExportTSDFToPcd_hashIndex(
                Objects::ITMScene<TVoxel,
                ITMVoxelBlockHash>* scene,
                const char *filename)
        {
            if(!scene){ return false; }

            ITMGlobalCache<TVoxel>* cache = scene->globalCache;

            bool hasGlobalCache = scene->useSwapping;


            TVoxel* voxelBlocks_global = 0;
            TVoxel* voxelBlocks_local = scene->localVBA.GetVoxelBlocks();

            if(hasGlobalCache){ voxelBlocks_global = cache->GetStoredVoxelBlock(0); }

            ORUtils::MemoryBlock<ITMHashEntry> hashTable(1, MEMORYDEVICE_CPU);
            scene->index.getEntriesCPUCopy(hashTable);
            ITMHashEntry* hash = hashTable.GetData(MEMORYDEVICE_CPU);


            int noEntries = hashTable.dataSize;

            ITMHashSwapState swapStateDefault;
            swapStateDefault.state = 2;
            std::vector<ITMHashSwapState> swapStates(noEntries, swapStateDefault);

            if(hasGlobalCache)
            {
                ITMSafeCall(cudaMemcpy(swapStates.data(), cache->GetSwapStates(true), noEntries*sizeof(ITMHashSwapState), cudaMemcpyDeviceToHost));
            }

            const size_t MAX_BLOCKS_TRANSFER_SIZE = 512;

            int noPoints = 0;
            std::vector<int> localPtr;
            std::vector<TVoxel> localVoxels_host;
            for(int i = 0; i < noEntries; i++)
            {
                if(hash[i].ptr >= -1 && swapStates[i].state != 2)
                {
                    noPoints += countVoxelsInBlock(voxelBlocks_global, i);
                }
                else if(hash[i].ptr >= 0 && swapStates[i].state == 2)
                {
                    localPtr.push_back(hash[i].ptr);
                }

                if(localPtr.size() >= MAX_BLOCKS_TRANSFER_SIZE || (i >= noEntries-1 && !localPtr.empty()))
                {
                    localVoxels_host.resize(localPtr.size() * SDF_BLOCK_SIZE3);

                    getVoxelsFromPtrList(localPtr.size(), voxelBlocks_local, localPtr.data(), localVoxels_host.data());

                    for(size_t j = 0; j < localPtr.size(); j++)
                    {
                        noPoints += countVoxelsInBlock(localVoxels_host.data(), j);
                    }

                    localPtr.clear();
                }
            }


            if(noPoints <= 0){ return false; }

            std::ofstream file(filename);
            if(!file.is_open()){ return false; }

            file << "VERSION 0.7\n";
            file << "FIELDS x y z " << (TVoxel::hasColorInformation? "rgb": "") << " intensity\n";
            file << "SIZE 4 4 4 " << (TVoxel::hasColorInformation? "4": "") << " 4\n";
            file << "TYPE F F F " << (TVoxel::hasColorInformation? "F": "") << " F\n";
            file << "COUNT 1 1 1 " << (TVoxel::hasColorInformation? "1": "") << " 1\n";
            file << "WIDTH " << noPoints << "\n";
            file << "HEIGHT 1\n";
            file << "VIEWPOINT 0 0 0 1 0 0 0\n";
            file << "POINTS " << noPoints << "\n";
            file << "DATA binary\n";


            std::vector<Vector3s> localPtrPos;
            for(int i = 0; i < noEntries; i++)
            {
                if(hash[i].ptr >= -1 && swapStates[i].state != 2)
                {
                    exportVoxelsInBlock(file, voxelBlocks_global, i, hash[i].pos);
                }
                else if(hash[i].ptr >= 0 && swapStates[i].state == 2)
                {
                    localPtr.push_back(hash[i].ptr);
                    localPtrPos.push_back(hash[i].pos);
                }

                if(localPtr.size() >= MAX_BLOCKS_TRANSFER_SIZE || (i >= noEntries-1 && !localPtr.empty()))
                {
                    localVoxels_host.resize(localPtr.size() * SDF_BLOCK_SIZE3);

                    getVoxelsFromPtrList(localPtr.size(), voxelBlocks_local, localPtr.data(), localVoxels_host.data());

                    for(size_t j = 0; j < localPtr.size(); j++)
                    {
                        exportVoxelsInBlock(file, localVoxels_host.data(), j, localPtrPos[j]);
                    }

                    localPtr.clear();
                    localPtrPos.clear();
                }
            }

            return true;
        }


        template<class TVoxel, class TIndex>
        bool ITMExportEngine_CUDA<TVoxel, TIndex>::ExportTSDFToPcd(const char *filename)
        { return false; }


        template <>
        bool ITMExportEngine_CUDA<ITMVoxel_s_rgb, ITMVoxelBlockHash>::ExportTSDFToPcd(const char *filename)
        {
            return ExportTSDFToPcd_hashIndex(scene_, filename);
        }

        template <>
        bool ITMExportEngine_CUDA<ITMVoxel_s, ITMVoxelBlockHash>::ExportTSDFToPcd(const char *filename)
        {
            return ExportTSDFToPcd_hashIndex(scene_, filename);
        }

        template <>
        bool ITMExportEngine_CUDA<ITMVoxel_f, ITMVoxelBlockHash>::ExportTSDFToPcd(const char *filename)
        {
            return ExportTSDFToPcd_hashIndex(scene_, filename);
        }

        template <>
        bool ITMExportEngine_CUDA<ITMVoxel_f_rgb, ITMVoxelBlockHash>::ExportTSDFToPcd(const char *filename)
        {
            return ExportTSDFToPcd_hashIndex(scene_, filename);
        }



    } // namespace
} // namespace