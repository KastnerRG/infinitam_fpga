// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Engine/ITMTracker.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		class ITMCompositeStepTracker : public ITMTracker
		{
		private:
			ITMTracker **trackers;
			int noTrackers;
			int currStep;
		public:

			void SetTracker(ITMTracker *tracker, int trackerId)
			{
				if (trackers[trackerId] != NULL) delete trackers[trackerId];
				trackers[trackerId] = tracker;
			}

			ITMCompositeStepTracker(int noTrackers)
			{
				trackers = new ITMTracker*[noTrackers];
				for (int i = 0; i < noTrackers; i++) trackers[i] = NULL;

				this->noTrackers = noTrackers;
				this->currStep = 0;
			}

			~ITMCompositeStepTracker(void)
			{
				for (int i = 0; i < noTrackers; i++)
					if (trackers[i] != NULL) delete trackers[i];

				delete [] trackers;
			}

			void TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
			{
				trackers[currStep]->TrackCamera(trackingState, view);
				currStep = (currStep + 1) % noTrackers;
			}

			void UpdateInitialPose(ITMTrackingState *trackingState)
			{
				for (int i = 0; i < noTrackers; i++) trackers[i]->UpdateInitialPose(trackingState);
			}

			void SetCurrentStep(int step){ currStep = step % noTrackers; }

			// Suppress the default copy constructor and assignment operator
			ITMCompositeStepTracker(const ITMCompositeTracker&);
			ITMCompositeStepTracker& operator=(const ITMCompositeTracker&);
		};
	}
}
