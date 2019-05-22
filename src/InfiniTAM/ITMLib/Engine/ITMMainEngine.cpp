// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"
#include "ITMIMUCalibrator_Tango.h"

#ifdef COMPILE_FOR_TANGO
#include "../../Engine/logging.h" // for debug
#endif

using namespace ITMLib::Engine;

ITMMainEngine::ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
//	if(settings->deviceType == ITMLibSettings::DEVICE_OPENCL)
//	{
//		ORUtils::OpenCLContext::getInstance().initialize(
//				(ORUtils::OpenCLContext::DeviceType)settings->openclDeviceType,
//				(int)settings->fpgaClAlgo);
//	}


	// create all the things required for marching cubes and mesh extraction
	// - uses additional memory (lots!)
	static const bool createMeshingEngine = true;

	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = settings;

	this->scene = new ITMScene<ITMVoxel, ITMVoxelIndex>(&(settings->sceneParams), settings->useSwapping, 
		settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);

	meshingEngine = NULL;
	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_OPENCL:
#ifdef COMPILE_WITH_OPENCL
	{
		ORUtils::OpenCLContext& clContext = ORUtils::OpenCLContext::getInstance();
		if(clContext.cl_context_.kernels[ORUtils::OpenCLContext::KERNEL_RAYCAST] ||
				clContext.hasAocxProgram(ORUtils::OpenCLContext::AOCX_RAYCAST) ||
				clContext.hasAocxProgram(ORUtils::OpenCLContext::AOCX_INTEGRATE_RAYCAST_ICP))
		{
			lowLevelEngine = new ITMLowLevelEngine_CPU();
			viewBuilder = new ITMViewBuilder_CPU(calib);
			visualisationEngine = new ITMVisualisationEngine_OpenCL<ITMVoxel, ITMVoxelIndex>(scene);
			if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
			break;
		}// else fall back to CPU
	}
#endif
	case ITMLibSettings::DEVICE_CPU:
		lowLevelEngine = new ITMLowLevelEngine_CPU();
		viewBuilder = new ITMViewBuilder_CPU(calib);
		visualisationEngine = new ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>(scene);
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		lowLevelEngine = new ITMLowLevelEngine_CUDA();
		viewBuilder = new ITMViewBuilder_CUDA(calib);
		visualisationEngine = new ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>(scene);
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CUDA<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		lowLevelEngine = new ITMLowLevelEngine_Metal();
		viewBuilder = new ITMViewBuilder_Metal(calib);
		visualisationEngine = new ITMVisualisationEngine_Metal<ITMVoxel, ITMVoxelIndex>(scene);
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	}

	visualisationEngine_freeView = NULL;

	mesh = NULL;
	if (createMeshingEngine) mesh = new ITMMesh(settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);


	Vector2i trackedImageSize = ITMTrackingController::GetTrackedImageSize(settings, imgSize_rgb, imgSize_d);

	renderState_live = visualisationEngine->CreateRenderState(trackedImageSize);
	renderState_freeview = NULL; //will be created by the visualisation engine

	renderState_live->rayCastInSceneIntegration =
			(settings->deviceType == ITMLibSettings::DEVICE_OPENCL) &&
			((settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_COMBINED) ||
					(settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_COMBINED_ICP));
#ifdef OPENCL_DEBUG
	renderState_live->raycastOnlyOpenCL =
			(settings->deviceType == ITMLibSettings::DEVICE_OPENCL) &&
			((settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_ICP) ||
					(settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_REPROGRAM) ||
					(settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_RAYCAST_ONLY));
#endif

#ifdef EXPERIMENT_INTEGRATE_RAYCAST_COMBINED
	renderState_live->rayCastInSceneIntegration =
			(settings->deviceType == ITMLibSettings::DEVICE_OPENCL
					|| settings->deviceType == ITMLibSettings::DEVICE_CPU)
					&& ((settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_COMBINED) ||
							(settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_COMBINED_ICP));
#endif

	denseMapper = new ITMDenseMapper<ITMVoxel, ITMVoxelIndex>(settings);
	denseMapper->ResetScene(scene);

	switch (settings->imuType)
	{
		case ITMLibSettings::IMU_IPAD:
			imuCalibrator = new ITMIMUCalibrator_iPad();
			break;
		case ITMLibSettings::IMU_TANGO:
			imuCalibrator = new ITMIMUCalibrator_Tango();
			break;
		default:
			imuCalibrator = new ITMIMUCalibrator_iPad();
	}


	tracker = ITMTrackerFactory<ITMVoxel, ITMVoxelIndex>::Instance().Make(trackedImageSize, settings, lowLevelEngine, imuCalibrator, scene);
	trackingController = new ITMTrackingController(tracker, visualisationEngine, lowLevelEngine, settings);

	trackingState = trackingController->BuildTrackingState(trackedImageSize);
	tracker->UpdateInitialPose(trackingState);


	exportEngine = NULL;
#ifndef COMPILE_WITHOUT_CUDA
	if(settings->deviceType == ITMLibSettings::DEVICE_CUDA)
	{
		exportEngine = new ITMExportEngine_CUDA<ITMVoxel, ITMVoxelIndex>(this->scene, this->trackingState);
	}
#endif
	if(settings->deviceType == ITMLibSettings::DEVICE_CPU || settings->deviceType == ITMLibSettings::DEVICE_OPENCL)
	{
		exportEngine = new ITMExportEngine_CPU<ITMVoxel, ITMVoxelIndex>(this->scene, this->trackingState);
	}


	view = NULL; // will be allocated by the view builder

	fusionActive = true;
	mainProcessingActive = true;
	numFrames = 0;

	createProfileTimers; // see NVTimer.h
}

ITMMainEngine::~ITMMainEngine()
{
	delete renderState_live;
	if (renderState_freeview!=NULL) delete renderState_freeview;

	delete scene;

	delete denseMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	if (view != NULL) delete view;

	delete visualisationEngine;
	if(visualisationEngine_freeView != NULL){ delete visualisationEngine_freeView; }

	if (meshingEngine != NULL) delete meshingEngine;

	if (mesh != NULL) delete mesh;

	if(exportEngine != NULL) delete exportEngine;
	
	deleteProfileTimers; // see NVTimer.h
}

ITMMesh* ITMMainEngine::UpdateMesh(void)
{
	if (mesh != NULL) meshingEngine->MeshScene(mesh, scene);
	return mesh;
}

void ITMMainEngine::SaveSceneToMesh(const char *objFileName)
{
	if (mesh == NULL) return;
	meshingEngine->MeshScene(mesh, scene);
	mesh->WriteSTL(objFileName);
}


bool ITMMainEngine::SaveTSDFToFile(const char *filename)
{
	if(exportEngine == NULL){ return false; }

	return exportEngine->ExportTSDFToPcd(filename);
}


bool ITMMainEngine::SavePosesToFile(const char *filename)
{
	if(exportEngine == NULL){ return false; }

	return exportEngine->ExportPoses(filename);
}


void ITMMainEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	// prepare image and turn it into a depth image
	if (imuMeasurement==NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter,settings->modelSensorNoise);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	if (!mainProcessingActive) return;

	// This is necessary to avoid slowing down / memory leak when using Intel OpenCL for FPGA
	if(settings->deviceType == ITMLibSettings::DEVICE_OPENCL && settings->openclDeviceType == ITMLibSettings::OPENCL_DEVICE_ACCELERATOR)
	{
		ORUtils::OpenCLContext::getInstance().reallocateQueues();
	}

	// tracking
	profileAndDisplayTime(trackingController->Track(trackingState, view), 0, "Tracking1");


	if(settings->trackerType == ITMLibSettings::TRACKER_IMU_INIT && numFrames > 0)
	{
		// raycast using IMU estimated pose
		profileAndDisplayTime(trackingController->Prepare(trackingState, view, renderState_live), 1, "Raycast");
		// Use ICP to refine the pose
		profileAndDisplayTime(trackingController->Track(trackingState, view), 2, "Tracking2");
	}

	// fusion
	if (fusionActive)
	{
		profileAndDisplayTime(denseMapper->ProcessFrame(view, trackingState, scene, renderState_live), 3, "Fusion");
		numFrames++;
	}

	// raycast to renderState_live for tracking and free visualisation
	if(settings->trackerType != ITMLibSettings::TRACKER_IMU_INIT)
	{
		profileAndDisplayTime(trackingController->Prepare(trackingState, view, renderState_live), 4, "Raycast");
	}

	// Save pose
	trackingState->SaveCurrentPose();
//
//	std::cout << trackingState->pose_d->GetM() << std::endl;
}

Vector2i ITMMainEngine::GetImageSize(void) const
{
	return renderState_live->raycastImage->noDims;
}

void ITMMainEngine::GetImage(ITMUChar4Image *out, GetImageType getImageType, ITMPose *pose, ITMIntrinsics *intrinsics)
{
	if (view == NULL) return;

	out->Clear();

	switch (getImageType)
	{
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) 
			out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth->noDims);
		if (settings->trackerType==ITMLib::Objects::ITMLibSettings::TRACKER_WICP)
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depthUncertainty->UpdateHostFromDevice();
			ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::WeightToUchar4(out, view->depthUncertainty);
		}
		else
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
			ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::DepthToUchar4(out, view->depth);
		}

		break;
	case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	{
		ORUtils::Image<Vector4u> *srcImage = renderState_live->raycastImage;
		out->ChangeDims(srcImage->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);	
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
	{
		if(visualisationEngine_freeView == NULL)
		{
			switch (settings->deviceType)
			{
			case ITMLibSettings::DEVICE_CPU:
			case ITMLibSettings::DEVICE_OPENCL:
					visualisationEngine_freeView = new ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>(scene);
					break;
				case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
					visualisationEngine_freeView = new ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>(scene);
#endif
					break;
				case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
					visualisationEngine_freeView = new ITMVisualisationEngine_Metal<ITMVoxel, ITMVoxelIndex>(scene);
#endif
					break;
			}
		}


		IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
		if (getImageType == ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
		else if (getImageType == ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
		if (renderState_freeview == NULL) renderState_freeview = visualisationEngine_freeView->CreateRenderState(out->noDims);

		visualisationEngine_freeView->FindVisibleBlocks(pose, intrinsics, renderState_freeview);
		visualisationEngine_freeView->CreateExpectedDepths(pose, intrinsics, renderState_freeview);
		visualisationEngine_freeView->RenderImage(pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN:
		break;
	};
}

void ITMMainEngine::turnOnIntegration() { fusionActive = true; }
void ITMMainEngine::turnOffIntegration() { fusionActive = false; }
void ITMMainEngine::turnOnMainProcessing() { mainProcessingActive = true; }
void ITMMainEngine::turnOffMainProcessing() { mainProcessingActive = false; }

void ITMMainEngine::ResetScene()
{
	denseMapper->ResetScene(this->scene);
	trackingState->poses.clear();
	numFrames = 0;
}
