// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibSettings.h"

#include <stdio.h>

using namespace ITMLib::Objects;

ITMLibSettings::ITMLibSettings(TrackerType tracker_type)
	: sceneParams(0.02f, 100, 0.005f, 0.2f, 3.0f, false)
{
	/// depth threshold for the ICP tracker
	depthTrackerICPThreshold = 0.1f * 0.1f;

	/// For ITMDepthTracker: ICP iteration termination threshold
	depthTrackerTerminationThreshold = 1e-3f;

	/// skips every other point when using the colour tracker
	skipPoints = true;

#ifndef COMPILE_WITHOUT_CUDA
	deviceType = DEVICE_CUDA;
#else
#ifdef COMPILE_WITH_METAL
	deviceType = DEVICE_METAL;
#else
	deviceType = DEVICE_CPU;
#endif
#endif

	//deviceType = DEVICE_CPU;

	openclDeviceType = OPENCL_DEVICE_CPU;

	openclAlgo = OPENCL_ALGO_INTEGRATE_ONLY;

	/// enables or disables swapping. HERE BE DRAGONS: It should work, but requires more testing
	useSwapping = false;

	/// enables or disables approximate raycast
	useApproximateRaycast = false;

	/// enable or disable bilateral depth filtering;
	useBilateralFilter = false;

	trackerType = tracker_type;

	/// model the sensor noise as  the weight for weighted ICP
	modelSensorNoise = false;
	if (trackerType == TRACKER_WICP) modelSensorNoise = true;
	

	// builds the tracking regime. level 0 is full resolution
	if (trackerType == TRACKER_IMU)
	{
		noHierarchyLevels = 2;
		trackingRegime = new TrackerIterationType[noHierarchyLevels];

		trackingRegime[0] = TRACKER_ITERATION_BOTH;
		trackingRegime[1] = TRACKER_ITERATION_TRANSLATION;
	    //trackingRegime[2] = TRACKER_ITERATION_TRANSLATION;
	}
	else if (trackerType == TRACKER_IMU_INIT)
	{
		noHierarchyLevels = 2;
		trackingRegime = new TrackerIterationType[noHierarchyLevels];

		trackingRegime[0] = TRACKER_ITERATION_BOTH;
		trackingRegime[1] = TRACKER_ITERATION_BOTH;
	}
	else
	{
		noHierarchyLevels = 5;
		trackingRegime = new TrackerIterationType[noHierarchyLevels];

		trackingRegime[0] = TRACKER_ITERATION_BOTH;
		trackingRegime[1] = TRACKER_ITERATION_BOTH;
		trackingRegime[2] = TRACKER_ITERATION_ROTATION; // Use TRANSLATION to make sure ICP code is correct
		trackingRegime[3] = TRACKER_ITERATION_ROTATION;
		trackingRegime[4] = TRACKER_ITERATION_ROTATION;
	}

	noICPIterations = new int[noHierarchyLevels];
	noICPIterations[0] = 2;
	for (int levelId = 1; levelId < noHierarchyLevels; levelId++)
	{
		noICPIterations[levelId] = noICPIterations[levelId - 1] + 2;
	}

	if (trackerType == TRACKER_REN) noICPRunTillLevel = 1;
	else noICPRunTillLevel = 0;

	if ((trackerType == TRACKER_COLOR) && (!ITMVoxel::hasColorInformation)) {
		printf("Error: Color tracker requires a voxel type with color information!\n");
	}

#ifdef COMPILE_FOR_TANGO
	imuType = IMU_TANGO;
#else
	imuType = IMU_IPAD;
#endif
}

ITMLibSettings::~ITMLibSettings()
{
	delete[] trackingRegime;
	delete[] noICPIterations;
}
