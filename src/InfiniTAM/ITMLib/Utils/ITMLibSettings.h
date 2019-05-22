// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMSceneParams.h"
#include "../Engine/ITMTracker.h"

namespace ITMLib
{
	namespace Objects
	{
		class ITMLibSettings
		{
		public:
			/// The device used to run the DeviceAgnostic code
			typedef enum {
				DEVICE_CPU,
				DEVICE_CUDA,
				DEVICE_METAL,
				DEVICE_OPENCL
			} DeviceType;

			/// Select the type of device to use
			DeviceType deviceType;

			/// The device to use when using OpenCL
			typedef enum {
				OPENCL_DEVICE_CPU,
				OPENCL_DEVICE_GPU,
				OPENCL_DEVICE_ACCELERATOR
			} OpenclDeviceType;

			/// Select the OpenCL device type
			OpenclDeviceType openclDeviceType;

			typedef enum {
				OPENCL_ALGO_INTEGRATE_ONLY,
				OPENCL_ALGO_RAYCAST_ONLY,
				OPENCL_ALGO_INTEGRATE_RAYCAST_REPROGRAM,
				OPENCL_ALGO_INTEGRATE_RAYCAST_COMBINED,
				OPENCL_ALGO_ICP,
				OPENCL_ALGO_INTEGRATE_RAYCAST_ICP,
				OPENCL_ALGO_COMBINED_ICP
			} OpenclAlgo;

			/// Set which algorithm to use when using OpenCL
			OpenclAlgo openclAlgo;

			/// Enables swapping between host and device.
			bool useSwapping;

			bool useApproximateRaycast;

			bool useBilateralFilter;

			bool modelSensorNoise;

			/// Tracker types
			typedef enum {
				//! Identifies a tracker based on colour image
				TRACKER_COLOR,
				//! Identifies a tracker based on depth image
				TRACKER_ICP,
				//! Identifies a tracker based on depth image (Ren et al, 2012)
				TRACKER_REN,
				//! Identifies a tracker based on depth image and IMU measurement
				TRACKER_IMU,
				//! Identifies a tracker based on IMU measurement only
				TRACKER_IMU_ONLY,
				//! Identifies a tracker that use weighted ICP only on depth image
				TRACKER_WICP,
				//! Identifies a tracker that use IMU data to initialize ICP
				TRACKER_IMU_INIT,
				//! Identifies a tracker based on external tracking (saved in IMU object)
				TRACKER_EXTERNAL_ONLY,
				//! Identifies a tracker based on external tracking (saved in IMU object) that use ICP only for data integration
				TRACKER_EXTERNAL_AND_ICP
			} TrackerType;

			/// Select the type of tracker to use
			TrackerType trackerType;

			/// The tracking regime used by the tracking controller
			TrackerIterationType *trackingRegime;

			/// The number of levels in the trackingRegime
			int noHierarchyLevels;
			
			/// Run ICP till # Hierarchy level, then switch to ITMRenTracker for local refinement.
			int noICPRunTillLevel;

			/// Number of ICP iterations per hierarchy level
			int* noICPIterations;

			/// For ITMColorTracker: skip every other point in energy function evaluation.
			bool skipPoints;

			/// For ITMDepthTracker: ICP distance threshold
			float depthTrackerICPThreshold;

			/// For ITMDepthTracker: ICP iteration termination threshold
			float depthTrackerTerminationThreshold;

			/// IMU types
			typedef enum {
				//! Identify an IMU as in an iPad device
				IMU_IPAD,
				//! Identify the IMU + visual odometry results on the Google Tango tablet
				IMU_TANGO
			} IMUType;

			/// Select the type of IMU to use
			IMUType imuType;

			/// Further, scene specific parameters such as voxel size
			ITMLib::Objects::ITMSceneParams sceneParams;

			ITMLibSettings(TrackerType tracker_type = TRACKER_ICP);
			~ITMLibSettings(void);

			// Suppress the default copy constructor and assignment operator
			ITMLibSettings(const ITMLibSettings&);
			ITMLibSettings& operator=(const ITMLibSettings&);
		};
	}
}
