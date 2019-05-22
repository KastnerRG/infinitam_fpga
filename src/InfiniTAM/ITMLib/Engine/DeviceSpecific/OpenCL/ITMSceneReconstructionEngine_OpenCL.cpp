// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSceneReconstructionEngine_OpenCL.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

#include "../../../../ORUtils/OpenCLContext.h"

#include "ITMSceneReconstructionEngine_OpenCL_knobs.h"


//#define DEPTH_FUSION_EXPORT_DATA

#ifdef DEPTH_FUSION_EXPORT_DATA
#include "IntegrateIntoSceneData.h"
#endif


using namespace ITMLib::Engine;


typedef struct
{
	cl_float4 data0;
	cl_float4 data1;
	cl_float4 data2;
	cl_float4 data3;

} Mat44_cl;




template<class TVoxel>
ITMSceneReconstructionEngine_OpenCL<TVoxel,ITMVoxelBlockHash>::ITMSceneReconstructionEngine_OpenCL(void):
	M_d_buffer(0), invM_d_buffer(0),
	pointsRay_buffer(0), projectedSdf_s_buffer(0)
{
	int noTotalEntries = ITMVoxelBlockHash::noTotalEntries;
	entriesAllocType = new ORUtils::MemoryBlock<unsigned char>(noTotalEntries, MEMORYDEVICE_CPU);
	blockCoords = new ORUtils::MemoryBlock<Vector4s>(noTotalEntries, MEMORYDEVICE_CPU);

#ifdef OPENCL_DEBUG
			visibleEntryIds_device_ = 0;
			depth_device_           = 0;
			localVBA_device_        = 0;
			hashTable_device_       = 0;
			M_d_device_             = 0;
			invM_d_device_          = 0;
			pointsRay_device_       = 0;
			projectedSdf_device_    = 0;
#endif
}

template<class TVoxel>
ITMSceneReconstructionEngine_OpenCL<TVoxel,ITMVoxelBlockHash>::~ITMSceneReconstructionEngine_OpenCL(void)
{
	delete entriesAllocType;
	delete blockCoords;
	if(pointsRay_buffer){ delete pointsRay_buffer; }
	if(projectedSdf_s_buffer){ delete projectedSdf_s_buffer; }
	if(M_d_buffer){ delete M_d_buffer; }
	if(invM_d_buffer){ delete invM_d_buffer; }

#ifdef OPENCL_DEBUG
	if(visibleEntryIds_device_){ clReleaseMemObject(visibleEntryIds_device_); }
	if(depth_device_)          { clReleaseMemObject(depth_device_); }
	if(localVBA_device_)       { clReleaseMemObject(localVBA_device_); }
	if(hashTable_device_)      { clReleaseMemObject(hashTable_device_); }
	if(M_d_device_)            { clReleaseMemObject(M_d_device_); }
	if(invM_d_device_)         { clReleaseMemObject(invM_d_device_); }
	if(pointsRay_device_)      { clReleaseMemObject(pointsRay_device_); }
	if(projectedSdf_device_)   { clReleaseMemObject(projectedSdf_device_); }
#endif
}

template<class TVoxel>
void ITMSceneReconstructionEngine_OpenCL<TVoxel,ITMVoxelBlockHash>::ResetScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
{
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();

	TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
	int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
	scene->localVBA.lastFreeBlockId = numBlocks - 1;

	ITMHashEntry tmpEntry;
	memset(&tmpEntry, 0, sizeof(ITMHashEntry));
	tmpEntry.ptr = -2;
	ITMHashEntry *hashEntry_ptr = scene->index.GetEntries();
	for (int i = 0; i < scene->index.noTotalEntries; ++i) hashEntry_ptr[i] = tmpEntry;
	int *excessList_ptr = scene->index.GetExcessAllocationList();
	for (int i = 0; i < SDF_EXCESS_LIST_SIZE; ++i) excessList_ptr[i] = i;

	scene->index.SetLastFreeExcessListId(SDF_EXCESS_LIST_SIZE - 1);


#ifdef OPENCL_DEBUG
	ORUtils::OpenCLContext::ClContext& clContext = ORUtils::OpenCLContext::getInstance().cl_context_;

	int err;
	int qid = ORUtils::OpenCLContext::KERNEL_INTEGRATE_SCENE;

	if(!localVBA_device_)
	{
		cl_mem* localVBA_device = const_cast<cl_mem*>(&localVBA_device_);
		*localVBA_device = clCreateBuffer(clContext.context, CL_MEM_READ_WRITE, sizeof(ITMVoxel_s) * scene->localVBA.allocatedSize, NULL, &err);
		if(!clCheckErr(err, "Failed to allocate memory!")){ return; }
		ORUtils::OpenCLContext::getInstance().voxelData_cl = *localVBA_device;
	}

	err = clEnqueueWriteBuffer(clContext.queues[qid], localVBA_device_, CL_TRUE, 0, sizeof(ITMVoxel_s) * scene->localVBA.allocatedSize, (void*)voxelBlocks_ptr, 0, NULL, NULL);
	if(!clCheckErr(err, "Failed to write data to the device!")){ return; }
#endif
}

template<class TVoxel>
void ITMSceneReconstructionEngine_OpenCL<TVoxel, ITMVoxelBlockHash>::IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	M_d = trackingState->pose_d->GetM();
	if (TVoxel::hasColorInformation) M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry *hashTable = scene->index.GetEntries();

	int *visibleEntryIds = renderState_vh->GetVisibleEntryIDs();
	int noVisibleEntries = renderState_vh->noVisibleEntries;

	bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;
	//bool approximateIntegration = !trackingState->requiresFullRendering;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int entryId = 0; entryId < noVisibleEntries; entryId++)
	{
		Vector3i globalPos;
		const ITMHashEntry &currentHashEntry = hashTable[visibleEntryIds[entryId]];

		if (currentHashEntry.ptr < 0) continue;

		globalPos.x = currentHashEntry.pos.x;
		globalPos.y = currentHashEntry.pos.y;
		globalPos.z = currentHashEntry.pos.z;
		globalPos *= SDF_BLOCK_SIZE;

		TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
		{
			Vector4f pt_model; int locId;

			locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

			if (stopIntegratingAtMaxW) if (localVoxelBlock[locId].w_depth == maxW) continue;
			//if (approximateIntegration) if (localVoxelBlock[locId].w_depth != 0) continue;

			pt_model.x = (float)(globalPos.x + x) * voxelSize;
			pt_model.y = (float)(globalPos.y + y) * voxelSize;
			pt_model.z = (float)(globalPos.z + z) * voxelSize;
			pt_model.w = 1.0f;

			ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel>::compute(localVoxelBlock[locId], pt_model, M_d, 
				projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, depthImgSize, rgb, rgbImgSize);
		}
	}
}

#ifdef COMPILE_WITH_OPENCL
// Have to put namespace here because of a bug in some versions of GCC
namespace ITMLib
{
	namespace Engine
	{
		template<>
		void ITMSceneReconstructionEngine_OpenCL<ITMVoxel_s, ITMVoxelBlockHash>::IntegrateIntoScene(ITMScene<ITMVoxel_s, ITMVoxelBlockHash> *scene, const ITMView *view,
			const ITMTrackingState *trackingState, const ITMRenderState *renderState)
		{
			ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

			const bool do_raycast = renderState_vh->rayCastInSceneIntegration;

			if(do_raycast)
			{
				ORUtils::OpenCLContext::getInstance().programAocxKernel(ORUtils::OpenCLContext::KERNEL_COMBINED);
			}
			else
			{
				ORUtils::OpenCLContext::getInstance().programAocxKernel(ORUtils::OpenCLContext::KERNEL_INTEGRATE_SCENE);
			}

			ORUtils::OpenCLContext::ClContext& clContext = ORUtils::OpenCLContext::getInstance().cl_context_;

			Vector2i depthImgSize  = view->depth->noDims;
			float voxelSize        = scene->sceneParams->voxelSize;
			float oneOverVoxelSize = 1.f / voxelSize;

			Matrix4f M_d    = trackingState->pose_d->GetM();
			Matrix4f invM_d = trackingState->pose_d->GetInvM();
			Vector4f projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;

			int noVisibleEntries = renderState_vh->noVisibleEntries;

			float mu = scene->sceneParams->mu;
			int maxW = scene->sceneParams->maxW;

#ifdef OPENCL_DEBUG
			int *visibleEntryIds = renderState_vh->GetVisibleEntryIDs();
			float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
			ITMVoxel_s *localVBA = scene->localVBA.GetVoxelBlocks();
			ITMHashEntry *hashTable = scene->index.GetEntries();
#endif

			bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;


			Matrix4f M_d_t = M_d.t(); // IMPORTANT: Using transpose because Matrix4f is stored columnwise
			Matrix4f invM_d_t = invM_d.t(); // IMPORTANT: Using transpose because Matrix4f is stored columnwise

			if(!M_d_buffer){ M_d_buffer = new ORUtils::MemoryBlock<Matrix4f>(1, MEMORYDEVICE_CPU); }
			if(!invM_d_buffer){ invM_d_buffer = new ORUtils::MemoryBlock<Matrix4f>(1, MEMORYDEVICE_CPU); }

			*M_d_buffer->GetData(MEMORYDEVICE_CPU)    = M_d_t;
			*invM_d_buffer->GetData(MEMORYDEVICE_CPU) = invM_d_t;

			cl_int2   depthImgSize_cl = *reinterpret_cast<cl_int2*>(&depthImgSize);
			cl_float4 projParams_d_cl = *reinterpret_cast<cl_float4*>(&projParams_d);


			uchar stopIntegratingAtMaxW_cl = stopIntegratingAtMaxW;


			// Memory for raycasting
			cl_mem pointsRay_device;
			cl_mem projectedSdf_device;
			Vector4f *pointsRay = 0;


			if(do_raycast)
			{
				if(!pointsRay_buffer){ pointsRay_buffer = new ORUtils::Image<Vector4f>(depthImgSize, MEMORYDEVICE_CPU); }
				if(!projectedSdf_s_buffer){ projectedSdf_s_buffer = new ORUtils::Image<short>(depthImgSize, MEMORYDEVICE_CPU); }

				pointsRay = pointsRay_buffer->GetData(MEMORYDEVICE_CPU);
				short *projectedSdf = projectedSdf_s_buffer->GetData(MEMORYDEVICE_CPU);
				for(int i = 0; i < depthImgSize.x*depthImgSize.y; i++)
				{
					pointsRay[i].w = 0.0f;
					projectedSdf[i] = ITMVoxel_s::SDF_initialValue();
				}

				pointsRay_device    = pointsRay_buffer->GetOpenCLData();
				projectedSdf_device = projectedSdf_s_buffer->GetOpenCLData();
			}




#ifdef DEPTH_FUSION_EXPORT_DATA
			reconstruct_hls::IntegrateIntoSceneData debug_data;

			debug_data.depthImgSize.x = depthImgSize.x;
			debug_data.depthImgSize.y = depthImgSize.y;
			debug_data.voxelSize = voxelSize;
			for(int i = 0; i < 4; i++)
			{
				debug_data.M_d.data0.at[i] = M_d_cl.data0.s[i];
				debug_data.M_d.data1.at[i] = M_d_cl.data1.s[i];
				debug_data.M_d.data2.at[i] = M_d_cl.data2.s[i];
				debug_data.M_d.data3.at[i] = M_d_cl.data3.s[i];
				debug_data.projParams_d.at[i] = projParams_d_cl.s[i];
			}
			debug_data.mu = mu;
			debug_data.maxW = maxW;
			debug_data.noVisibleEntries = noVisibleEntries;
			debug_data.stopIntegratingAtMaxW = stopIntegratingAtMaxW;
			debug_data.setArrays(
					(reconstruct_hls::ITMVoxel_s*)scene->localVBA.GetVoxelBlocks(), scene->localVBA.allocatedSize, (reconstruct_hls::ITMHashEntry*)scene->index.GetEntries(), scene->index.noTotalEntries,
					renderState_vh->GetVisibleEntryIDs(), noVisibleEntries, view->depth->GetData(MEMORYDEVICE_CPU), depthImgSize.x, depthImgSize.y);
#endif





			int err;
			int qid = do_raycast?
					ORUtils::OpenCLContext::KERNEL_COMBINED:
					ORUtils::OpenCLContext::KERNEL_INTEGRATE_SCENE;
			int kid = do_raycast?
					ORUtils::OpenCLContext::KERNEL_COMBINED:
					ORUtils::OpenCLContext::KERNEL_INTEGRATE_SCENE;

#ifdef OPENCL_DEBUG
			if(!visibleEntryIds_device_)
			{
				cl_mem* visibleEntryIds_device = const_cast<cl_mem*>(&visibleEntryIds_device_);
				*visibleEntryIds_device = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(int)*SDF_LOCAL_BLOCK_NUM, NULL, &err);
				if(!clCheckErr(err, "Failed to allocate memory!")){ return; }
			}
			if(!depth_device_)
			{
				cl_mem* depth_device = const_cast<cl_mem*>(&depth_device_);
				*depth_device = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(float)*depthImgSize.x*depthImgSize.y, NULL, &err);
				if(!clCheckErr(err, "Failed to allocate memory!")){ return; }
			}
			if(!hashTable_device_)
			{
				cl_mem* hashTable_device = const_cast<cl_mem*>(&hashTable_device_);
				*hashTable_device = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(ITMHashEntry) * scene->index.noTotalEntries, NULL, &err);
				if(!clCheckErr(err, "Failed to allocate memory!")){ return; }
			}
			if(!M_d_device_)
			{
				cl_mem* M_d_device = const_cast<cl_mem*>(&M_d_device_);
				*M_d_device = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(Matrix4f), NULL, &err);
				if(!clCheckErr(err, "Failed to allocate memory!")){ return; }
			}

			err = clEnqueueWriteBuffer(clContext.queues[qid], visibleEntryIds_device_, CL_FALSE, 0, sizeof(int)*noVisibleEntries, (void*)visibleEntryIds, 0, NULL, NULL);
			if(!clCheckErr(err, "Failed to write data to the device!")){ return; }
			err = clEnqueueWriteBuffer(clContext.queues[qid], depth_device_, CL_FALSE, 0, sizeof(float)*depthImgSize.x*depthImgSize.y, (void*)depth, 0, NULL, NULL);
			if(!clCheckErr(err, "Failed to write data to the device!")){ return; }
			err = clEnqueueWriteBuffer(clContext.queues[qid], hashTable_device_, CL_FALSE, 0, sizeof(ITMHashEntry) * scene->index.noTotalEntries, (void*)hashTable, 0, NULL, NULL);
			if(!clCheckErr(err, "Failed to write data to the device!")){ return; }
			err = clEnqueueWriteBuffer(clContext.queues[qid], M_d_device_, CL_FALSE, 0, sizeof(Matrix4f), (void*)&M_d_t, 0, NULL, NULL);
			if(!clCheckErr(err, "Failed to write data to the device!")){ return; }

			if(do_raycast)
			{
				short *projectedSdf = projectedSdf_s_buffer->GetData(MEMORYDEVICE_CPU);

				if(!invM_d_device_)
				{
					cl_mem* invM_d_device = const_cast<cl_mem*>(&invM_d_device_);
					*invM_d_device = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(Matrix4f), NULL, &err);
					if(!clCheckErr(err, "Failed to allocate memory!")){ return; }
				}
				if(!pointsRay_device_)
				{
					cl_mem* pointsRay_device = const_cast<cl_mem*>(&pointsRay_device_);
					*pointsRay_device = clCreateBuffer(clContext.context, CL_MEM_READ_WRITE, sizeof(Vector4f)*depthImgSize.x*depthImgSize.y, NULL, &err);
					if(!clCheckErr(err, "Failed to allocate memory!")){ return; }
				}
				if(!projectedSdf_device_)
				{
					cl_mem* projectedSdf_device = const_cast<cl_mem*>(&projectedSdf_device_);
					*projectedSdf_device = clCreateBuffer(clContext.context, CL_MEM_READ_WRITE, sizeof(short)*depthImgSize.x*depthImgSize.y, NULL, &err);
					if(!clCheckErr(err, "Failed to allocate memory!")){ return; }
				}

				err = clEnqueueWriteBuffer(clContext.queues[qid], invM_d_device_, CL_FALSE, 0, sizeof(Matrix4f), (void*)&invM_d_t, 0, NULL, NULL);
				if(!clCheckErr(err, "Failed to write data to the device!")){ return; }

				err = clEnqueueWriteBuffer(clContext.queues[qid], pointsRay_device_, CL_FALSE, 0, sizeof(Vector4f)*depthImgSize.x*depthImgSize.y, (void*)pointsRay, 0, NULL, NULL);
				if(!clCheckErr(err, "Failed to write data to the device!")){ return; }

				err = clEnqueueWriteBuffer(clContext.queues[qid], projectedSdf_device_, CL_FALSE, 0, sizeof(short)*depthImgSize.x*depthImgSize.y, (void*)projectedSdf, 0, NULL, NULL);
				if(!clCheckErr(err, "Failed to write data to the device!")){ return; }

				pointsRay_device              = pointsRay_device_;
				projectedSdf_device           = projectedSdf_device_;
			}


			cl_mem visibleEntryIds_device = visibleEntryIds_device_;
			cl_mem depth_device           = depth_device_;
			cl_mem localVBA_device        = localVBA_device_;
			cl_mem hashTable_device       = hashTable_device_;
			cl_mem M_d_device             = M_d_device_;
			cl_mem invM_d_device          = invM_d_device_;

#else

			cl_mem visibleEntryIds_device = renderState_vh->GetVisibleEntryIDs_cl();
			cl_mem depth_device           = view->depth->GetOpenCLData();
			cl_mem localVBA_device        = scene->localVBA.GetVoxelBlocks_cl();
			cl_mem hashTable_device       = scene->index.GetEntries_cl();
			cl_mem M_d_device             = M_d_buffer->GetOpenCLData();
			cl_mem invM_d_device          = invM_d_buffer->GetOpenCLData();
#endif


			size_t localWorkSize[3];
			localWorkSize[0] = do_raycast? WORK_GROUP_SIZE_X3: WORK_GROUP_SIZE_X;
			localWorkSize[1] = do_raycast? WORK_GROUP_SIZE_Y3: WORK_GROUP_SIZE_Y;
			localWorkSize[2] = do_raycast? WORK_GROUP_SIZE_Z3: WORK_GROUP_SIZE_Z;

			size_t workSize[3];
			if(!do_raycast)
			{
#if   KNOB_ENTRYID_TYPE == WORKGROUP
				workSize[0] = (unsigned int)noVisibleEntries * localWorkSize[0];
				workSize[1] = localWorkSize[1];
				workSize[2] = localWorkSize[2];
#elif KNOB_ENTRYID_TYPE == WORKITEMS
				workSize[0] = (unsigned int)ceil(float(noVisibleEntries) / localWorkSize[0]) * localWorkSize[0];
				workSize[1] = localWorkSize[1];
				workSize[2] = localWorkSize[2];
#elif KNOB_ENTRYID_TYPE == LOOP
				workSize[0] = localWorkSize[0];
				workSize[1] = localWorkSize[1];
				workSize[2] = localWorkSize[2];
#endif
			}
			else
			{
#if KNOB3_ENTRYID_TYPE == LOOP
				workSize[0] = localWorkSize[0];
				workSize[1] = localWorkSize[1];
				workSize[2] = localWorkSize[2];
#else
#error
#endif
			}

			int argid = 0;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_int2), (void*)&depthImgSize_cl); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(float),   (void*)&voxelSize); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem),  (void*)&M_d_device); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_float4), (void*)&projParams_d_cl); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(float),  (void*)&mu); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(int),    (void*)&maxW); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem), (void*)&depth_device); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem), (void*)&localVBA_device); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem), (void*)&hashTable_device); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem), (void*)&visibleEntryIds_device); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(int),    (void*)&noVisibleEntries); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(uchar),  (void*)&stopIntegratingAtMaxW_cl); argid++;
			if(do_raycast)
			{
				clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem), (void*)&pointsRay_device); argid++;
				clSetKernelArg(clContext.kernels[kid], argid, sizeof(float), (void*)&oneOverVoxelSize); argid++;
				clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem),  (void*)&invM_d_device); argid++;
#if !KNOB3_PROJECTED_SDF_LOCAL
				clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem), (void*)&projectedSdf_device); argid++;
#endif
			}

			err = clEnqueueNDRangeKernel(
					clContext.queues[qid], clContext.kernels[kid], 3, NULL,
					workSize, localWorkSize, 0, NULL, &clContext.events[0]);
#ifndef OPENCL_DEBUG
			clFinish(clContext.queues[qid]);
#endif
			if(!clCheckErr(err, "Failed to execute kernel!")){ return; }





#ifdef OPENCL_DEBUG
			
			if(!renderState_vh->raycastOnlyOpenCL && !do_raycast)
			{
				err = clEnqueueReadBuffer(
						clContext.queues[qid], localVBA_device_, CL_TRUE, 0,
						sizeof(ITMVoxel_s) * scene->localVBA.allocatedSize, (void*)localVBA, 0, NULL, NULL);
				if(!clCheckErr(err, "Failed to read data from the device!")){ return; }
			}
			else if(!do_raycast)
			{
				clFinish(clContext.queues[qid]);
			}

			if(do_raycast)
			{
				err = clEnqueueReadBuffer(
						clContext.queues[qid], pointsRay_device_, CL_TRUE, 0,
						sizeof(Vector4f)*depthImgSize.x*depthImgSize.y, (void*)pointsRay, 0, NULL, NULL);
				if(!clCheckErr(err, "Failed to read data from the device!")){ return; }
			}


			// ----
			// Kernel profiling with OpenCL
			//-----
			cl_ulong k_time_start, k_time_end;
			clGetEventProfilingInfo(clContext.events[0], CL_PROFILING_COMMAND_START, sizeof(k_time_start), &k_time_start, NULL);
			clGetEventProfilingInfo(clContext.events[0], CL_PROFILING_COMMAND_END, sizeof(k_time_end), &k_time_end, NULL);
			double kernel_time = (k_time_end - k_time_start) / 1000000.0;
			std::cout << "Fusion_integrate_Kernel: " << kernel_time << "ms\n";
#endif


			Vector4f* pointsRay_out = 0;

			if(do_raycast)
			{
				// Raycast
				// Nearest neighbor interpolation
				pointsRay_out = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
				for(int y = 0; y < depthImgSize.y; y++)
					for(int x = 0; x < depthImgSize.x; x++)
					{
						int depthIdx = x + y * depthImgSize.x;
						bool valueSet = false;
						if(pointsRay[depthIdx].w == 0.0f)
						{
							for(int a = -1; a <= 1 && !valueSet; a++)
							{
								for(int b = -1; b <= 1; b++)
								{
									int index = (y+a) * depthImgSize.x + x+b;
									if(index >= 0 && index < depthImgSize.x * depthImgSize.y)
									{
										if(pointsRay[index].w != 0.0f)
										{
											pointsRay_out[depthIdx] = pointsRay[index];
											valueSet = true;
											break;
										}
									}
								}
							}
						}

						if(!valueSet)
						{
							pointsRay_out[depthIdx] = pointsRay[depthIdx];
						}
					}
			}




#ifdef DEPTH_FUSION_EXPORT_DATA
			debug_data.setOutput(
					(reconstruct_hls::ITMVoxel_s*)scene->localVBA.GetVoxelBlocks(),
					(reconstruct_hls::float4*)pointsRay_out);

			debug_data.writeInputs("inputs.dat");
			debug_data.writeOutput("output.dat");
#endif

		}
	}
}
#endif // COMPILE_WITH_OPENCL

template<class TVoxel>
void ITMSceneReconstructionEngine_OpenCL<TVoxel, ITMVoxelBlockHash>::AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState, bool onlyUpdateVisibleList)
{
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, invM_d;
	Vector4f projParams_d, invProjParams_d;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	M_d = trackingState->pose_d->GetM(); M_d.inv(invM_d);

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	invProjParams_d = projParams_d;
	invProjParams_d.x = 1.0f / invProjParams_d.x;
	invProjParams_d.y = 1.0f / invProjParams_d.y;

	float mu = scene->sceneParams->mu;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	ITMHashEntry *hashTable = scene->index.GetEntries();
	ITMHashSwapState *swapStates = scene->useSwapping ? scene->globalCache->GetSwapStates(false) : 0;
	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType();
	uchar *entriesAllocType = this->entriesAllocType->GetData(MEMORYDEVICE_CPU);
	Vector4s *blockCoords = this->blockCoords->GetData(MEMORYDEVICE_CPU);
	int noTotalEntries = scene->index.noTotalEntries;

	bool useSwapping = scene->useSwapping;

	float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);

	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();

	int noVisibleEntries = 0;

	memset(entriesAllocType, 0, noTotalEntries);

	for (int i = 0; i < renderState_vh->noVisibleEntries; i++)
		entriesVisibleType[visibleEntryIDs[i]] = 3; // visible at previous frame and unstreamed

	//build hashVisibility
#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int locId = 0; locId < depthImgSize.x*depthImgSize.y; locId++)
	{
		int y = locId / depthImgSize.x;
		int x = locId - y * depthImgSize.x;
		buildHashAllocAndVisibleTypePP(entriesAllocType, entriesVisibleType, x, y, blockCoords, depth, invM_d,
			invProjParams_d, mu, depthImgSize, oneOverVoxelSize, hashTable, scene->sceneParams->viewFrustum_min,
			scene->sceneParams->viewFrustum_max);
	}

	if (onlyUpdateVisibleList) useSwapping = false;
	if (!onlyUpdateVisibleList)
	{
		//allocate
		for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
		{
			int vbaIdx, exlIdx;
			unsigned char hashChangeType = entriesAllocType[targetIdx];

			switch (hashChangeType)
			{
			case 1: //needs allocation, fits in the ordered list
				vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;

				if (vbaIdx >= 0) //there is room in the voxel block array
				{
					Vector4s pt_block_all = blockCoords[targetIdx];

					ITMHashEntry hashEntry;
					hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
					hashEntry.ptr = voxelAllocationList[vbaIdx];
					hashEntry.offset = 0;

					hashTable[targetIdx] = hashEntry;
				}

				break;
			case 2: //needs allocation in the excess list
				vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
				exlIdx = lastFreeExcessListId; lastFreeExcessListId--;

				if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
				{
					Vector4s pt_block_all = blockCoords[targetIdx];

					ITMHashEntry hashEntry;
					hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
					hashEntry.ptr = voxelAllocationList[vbaIdx];
					hashEntry.offset = 0;

					int exlOffset = excessAllocationList[exlIdx];

					hashTable[targetIdx].offset = exlOffset + 1; //connect to child

					hashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list

					entriesVisibleType[SDF_BUCKET_NUM + exlOffset] = 1; //make child visible and in memory
				}

				break;
			}
		}
	}

	//build visible list
	for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
	{
		unsigned char hashVisibleType = entriesVisibleType[targetIdx];
		const ITMHashEntry &hashEntry = hashTable[targetIdx];
		
		if (hashVisibleType == 3)
		{
			bool isVisibleEnlarged, isVisible;

			if (useSwapping)
			{
				checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
				if (!isVisibleEnlarged) hashVisibleType = 0;
			} else {
				checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
				if (!isVisible) { hashVisibleType = 0; }
			}
			entriesVisibleType[targetIdx] = hashVisibleType;
		}

		if (useSwapping)
		{
			if (hashVisibleType > 0 && swapStates[targetIdx].state != 2) swapStates[targetIdx].state = 1;
		}

		if (hashVisibleType > 0)
		{	
			visibleEntryIDs[noVisibleEntries] = targetIdx;
			noVisibleEntries++;
		}

#if 0
		// "active list", currently disabled
		if (hashVisibleType == 1)
		{
			activeEntryIDs[noActiveEntries] = targetIdx;
			noActiveEntries++;
		}
#endif
	}

	//reallocate deleted ones from previous swap operation
	if (useSwapping)
	{
		for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
		{
			int vbaIdx;
			ITMHashEntry hashEntry = hashTable[targetIdx];

			if (entriesVisibleType[targetIdx] > 0 && hashEntry.ptr == -1) 
			{
				vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
				if (vbaIdx >= 0) hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
			}
		}
	}

	renderState_vh->noVisibleEntries = noVisibleEntries;

	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	scene->index.SetLastFreeExcessListId(lastFreeExcessListId);
}

template<class TVoxel>
ITMSceneReconstructionEngine_OpenCL<TVoxel,ITMPlainVoxelArray>::ITMSceneReconstructionEngine_OpenCL(void)
{}

template<class TVoxel>
ITMSceneReconstructionEngine_OpenCL<TVoxel,ITMPlainVoxelArray>::~ITMSceneReconstructionEngine_OpenCL(void)
{}

template<class TVoxel>
void ITMSceneReconstructionEngine_OpenCL<TVoxel,ITMPlainVoxelArray>::ResetScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene)
{
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();

	TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
	int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
	scene->localVBA.lastFreeBlockId = numBlocks - 1;
}

template<class TVoxel>
void ITMSceneReconstructionEngine_OpenCL<TVoxel, ITMPlainVoxelArray>::AllocateSceneFromDepth(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState, bool onlyUpdateVisibleList)
{}

template<class TVoxel>
void ITMSceneReconstructionEngine_OpenCL<TVoxel, ITMPlainVoxelArray>::IntegrateIntoScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	M_d = trackingState->pose_d->GetM();
	if (TVoxel::hasColorInformation) M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel *voxelArray = scene->localVBA.GetVoxelBlocks();

	const ITMPlainVoxelArray::IndexData *arrayInfo = scene->index.getIndexData();

	bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;
	//bool approximateIntegration = !trackingState->requiresFullRendering;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int locId = 0; locId < scene->index.getVolumeSize().x*scene->index.getVolumeSize().y*scene->index.getVolumeSize().z; ++locId)
	{
		int z = locId / (scene->index.getVolumeSize().x*scene->index.getVolumeSize().y);
		int tmp = locId - z * scene->index.getVolumeSize().x*scene->index.getVolumeSize().y;
		int y = tmp / scene->index.getVolumeSize().x;
		int x = tmp - y * scene->index.getVolumeSize().x;
		Vector4f pt_model;

		if (stopIntegratingAtMaxW) if (voxelArray[locId].w_depth == maxW) continue;
		//if (approximateIntegration) if (voxelArray[locId].w_depth != 0) continue;

		pt_model.x = (float)(x + arrayInfo->offset.x) * voxelSize;
		pt_model.y = (float)(y + arrayInfo->offset.y) * voxelSize;
		pt_model.z = (float)(z + arrayInfo->offset.z) * voxelSize;
		pt_model.w = 1.0f;

		ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel>::compute(voxelArray[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW, 
			depth, depthImgSize, rgb, rgbImgSize);
	}
}

template class ITMLib::Engine::ITMSceneReconstructionEngine_OpenCL<ITMVoxel, ITMVoxelIndex>;
