// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMTrackingController.h"

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

void ITMTrackingController::Track(ITMTrackingState *trackingState, const ITMView *view)
{
	if (trackingState->age_pointCloud!=-1) tracker->TrackCamera(trackingState, view);

#ifdef COMPILE_WITH_OPENCL
#ifdef OPENCL_DEBUG
	ITMDepthTracker_OpenCL* cl_tracker = dynamic_cast<ITMDepthTracker_OpenCL*>(tracker);
	if(cl_tracker)
	{
		if(cl_tracker->time_kernels > 0.0)
		{ std::cout << "Tracking1_Kernel: " << cl_tracker->time_kernels << "ms\n"; }
		cl_tracker->time_kernels = 0;
	}
#endif
#endif

	trackingState->requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;
}

void ITMTrackingController::Prepare(ITMTrackingState *trackingState, const ITMView *view, ITMRenderState *renderState)
{
	//render for tracking

	if (settings->trackerType == ITMLibSettings::TRACKER_COLOR)
	{
		ITMPose pose_rgb(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
		if(!renderState->rayCastInSceneIntegration)
		{
			profileAndDisplayTime(
					visualisationEngine->CreateExpectedDepths(&pose_rgb, &(view->calib->intrinsics_rgb), renderState),
					0, "Raycast_CreateDepths");
		}
		profileAndDisplayTime(
				visualisationEngine->CreatePointCloud(view, trackingState, renderState, settings->skipPoints),
				1, "Raycast_CreatePointCloud");
		trackingState->age_pointCloud = 0;
	}
	else
	{
		if(!renderState->rayCastInSceneIntegration)
		{
			profileAndDisplayTime(
					visualisationEngine->CreateExpectedDepths(trackingState->pose_d, &(view->calib->intrinsics_d), renderState),
					0, "Raycast_CreateDepths");
		}

		if (trackingState->requiresFullRendering)
		{
			profileAndDisplayTime(
					visualisationEngine->CreateICPMaps(view, trackingState, renderState),
					2, "Raycast_CreateICPMaps");
			trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
			if (trackingState->age_pointCloud==-1) trackingState->age_pointCloud=-2;
			else trackingState->age_pointCloud = 0;
		}
		else
		{
			profileAndDisplayTime(
					visualisationEngine->ForwardRender(view, trackingState, renderState),
					3, "Raycast_ForwardRender");
			trackingState->age_pointCloud++;
		}
	}
}
