// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMRenderState.h"

#include "../Engine/ITMVisualisationEngine.h"
#include "../Engine/ITMLowLevelEngine.h"

#include "ITMTrackerFactory.h"

#include "../../Utils/NVTimer.h"

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		*/
		class ITMTrackingController
		{
		private:
			const ITMLibSettings *settings;
			const IITMVisualisationEngine *visualisationEngine;
			const ITMLowLevelEngine *lowLevelEngine;

			ITMTracker *tracker;

			MemoryDeviceType memoryType;

			declareProfileTimers(4); // see NVTimer.h


		public:
			void Track(ITMTrackingState *trackingState, const ITMView *view);
			void Prepare(ITMTrackingState *trackingState, const ITMView *view, ITMRenderState *renderState);

			ITMTrackingController(ITMTracker *tracker, const IITMVisualisationEngine *visualisationEngine, const ITMLowLevelEngine *lowLevelEngine,
				const ITMLibSettings *settings)
			{
				this->tracker = tracker;
				this->settings = settings;
				this->visualisationEngine = visualisationEngine;
				this->lowLevelEngine = lowLevelEngine;

				memoryType = settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;

				createProfileTimers; // see NVTimer.h
			}
			
			~ITMTrackingController()
			{
				deleteProfileTimers; // see NVTimer.h
			}

			ITMTrackingState *BuildTrackingState(const Vector2i & trackedImageSize) const
			{
				return new ITMTrackingState(trackedImageSize, memoryType);
			}

			static Vector2i GetTrackedImageSize(const ITMLibSettings *settings, const Vector2i& imgSize_rgb, const Vector2i& imgSize_d)
			{
				return settings->trackerType == ITMLibSettings::TRACKER_COLOR ? imgSize_rgb : imgSize_d;
			}

			// Suppress the default copy constructor and assignment operator
			ITMTrackingController(const ITMTrackingController&);
			ITMTrackingController& operator=(const ITMTrackingController&);
		};
	}
}
