
#include "ITMExportEngine_CPU.h"

#include <fstream>
#include <vector>


using namespace ITMLib::Engine;
using namespace ITMLib::Objects;



namespace ITMLib
{
    namespace Engine
    {
        template<class TVoxel, class TIndex>
        int ITMExportEngine_CPU<TVoxel, TIndex>::countVoxelsInBlock(const TVoxel* voxelBlocks, int block_ptr)
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
        void ITMExportEngine_CPU<TVoxel, TIndex>::exportVoxelsInBlock(std::ofstream& file, const TVoxel* voxelBlocks, int block_ptr, Vector3s blockPos)
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
        bool ITMExportEngine_CPU<TVoxel, TIndex>::ExportTSDFToPcd_hashIndex(Objects::ITMScene<TVoxel, ITMVoxelBlockHash>* scene, const char *filename)
        {
            if(!scene){ return false; }

            ITMGlobalCache<TVoxel>* cache = scene->globalCache;

            bool hasGlobalCache = scene->useSwapping;


            TVoxel* voxelBlocks_global = 0;
            TVoxel* voxelBlocks_local = scene->localVBA.GetVoxelBlocks();

            if(hasGlobalCache){ voxelBlocks_global = cache->GetStoredVoxelBlock(0); }

            ITMHashEntry* hash = scene->index.GetEntries();

            int noEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;

            ITMHashSwapState swapStateDefault;
            swapStateDefault.state = 2;
            std::vector<ITMHashSwapState> swapStates(noEntries, swapStateDefault);

            if(hasGlobalCache)
            {
                memcpy(swapStates.data(), cache->GetSwapStates(false), noEntries*sizeof(ITMHashSwapState));
            }


            int noPoints = 0;
            for(int i = 0; i < noEntries; i++)
            {
                if(hash[i].ptr >= -1 && swapStates[i].state != 2)
                {
                    noPoints += countVoxelsInBlock(voxelBlocks_global, i);
                }
                else if(hash[i].ptr >= 0 && swapStates[i].state == 2)
                {
                    noPoints += countVoxelsInBlock(voxelBlocks_local, hash[i].ptr);
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


            for(int i = 0; i < noEntries; i++)
            {
                if(hash[i].ptr >= -1 && swapStates[i].state != 2)
                {
                    exportVoxelsInBlock(file, voxelBlocks_global, i, hash[i].pos);
                }
                else if(hash[i].ptr >= 0 && swapStates[i].state == 2)
                {
                    exportVoxelsInBlock(file, voxelBlocks_local, hash[i].ptr, hash[i].pos);
                }
            }


            return true;
        }



        template<class TVoxel, class TIndex>
        bool ITMExportEngine_CPU<TVoxel, TIndex>::ExportTSDFToPcd(const char *filename)
        { return false; }


        template <>
        bool ITMExportEngine_CPU<ITMVoxel_s_rgb, ITMVoxelBlockHash>::ExportTSDFToPcd(const char *filename)
        {
            return ExportTSDFToPcd_hashIndex(scene_, filename);
        }

        template <>
        bool ITMExportEngine_CPU<ITMVoxel_s, ITMVoxelBlockHash>::ExportTSDFToPcd(const char *filename)
        {
            return ExportTSDFToPcd_hashIndex(scene_, filename);
        }

        template <>
        bool ITMExportEngine_CPU<ITMVoxel_f, ITMVoxelBlockHash>::ExportTSDFToPcd(const char *filename)
        {
            return ExportTSDFToPcd_hashIndex(scene_, filename);
        }

        template <>
        bool ITMExportEngine_CPU<ITMVoxel_f_rgb, ITMVoxelBlockHash>::ExportTSDFToPcd(const char *filename)
        {
            return ExportTSDFToPcd_hashIndex(scene_, filename);
        }


    } // namespace
} // namespace
