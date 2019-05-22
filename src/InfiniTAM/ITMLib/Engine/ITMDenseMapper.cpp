// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDenseMapper.h"

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel, TIndex>::ITMDenseMapper(const ITMLibSettings *settings)
{
	swappingEngine = NULL;

	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_OPENCL:
#ifdef COMPILE_WITH_OPENCL
	{
		ORUtils::OpenCLContext& clContext = ORUtils::OpenCLContext::getInstance();
		if(     clContext.cl_context_.kernels[ORUtils::OpenCLContext::KERNEL_INTEGRATE_SCENE] ||
				clContext.cl_context_.kernels[ORUtils::OpenCLContext::KERNEL_COMBINED] ||
				clContext.hasAocxProgram(ORUtils::OpenCLContext::AOCX_INTEGRATE_SCENE) ||
				clContext.hasAocxProgram(ORUtils::OpenCLContext::AOCX_COMBINED) ||
				clContext.hasAocxProgram(ORUtils::OpenCLContext::AOCX_INTEGRATE_RAYCAST_ICP) ||
				clContext.hasAocxProgram(ORUtils::OpenCLContext::AOCX_COMBINED_ICP))
		{
			sceneRecoEngine = new ITMSceneReconstructionEngine_OpenCL<TVoxel,TIndex>();
			if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<TVoxel,TIndex>();
			break;
		}// else fall back to CPU
	}
#endif
	case ITMLibSettings::DEVICE_CPU:
		sceneRecoEngine = new ITMSceneReconstructionEngine_CPU<TVoxel,TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<TVoxel,TIndex>();
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		sceneRecoEngine = new ITMSceneReconstructionEngine_CUDA<TVoxel,TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CUDA<TVoxel,TIndex>();
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		sceneRecoEngine = new ITMSceneReconstructionEngine_Metal<TVoxel, TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<TVoxel, TIndex>();
#endif
		break;
	}

	createProfileTimers; // see NVTimer.h
}

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel,TIndex>::~ITMDenseMapper()
{
	delete sceneRecoEngine;
	if (swappingEngine!=NULL) delete swappingEngine;

	deleteProfileTimers; // see NVTimer.h
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::ResetScene(ITMScene<TVoxel,TIndex> *scene)
{
	sceneRecoEngine->ResetScene(scene);
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState)
{
	// allocation
	profileAndDisplayTime(
			sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState),
			0, "Fusion_allocate");


	// integration
	profileAndDisplayTime(
			sceneRecoEngine->IntegrateIntoScene(scene, view, trackingState, renderState),
			1, "Fusion_integrate");


	if (swappingEngine != NULL) {
		// swapping: CPU -> GPU
		swappingEngine->IntegrateGlobalIntoLocal(scene, renderState);
		// swapping: GPU -> CPU
		swappingEngine->SaveToGlobalMemory(scene, renderState);
	}
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState)
{
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState, true);
}

template class ITMLib::Engine::ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;
