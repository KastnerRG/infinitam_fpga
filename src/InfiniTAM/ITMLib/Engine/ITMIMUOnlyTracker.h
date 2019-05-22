//
// Created by qkgautier on 7/31/16.
//

#pragma once

#include "../Objects/ITMIMUMeasurement.h"
#include "../Engine/ITMTracker.h"
#include "../Engine/ITMIMUCalibrator.h"


namespace ITMLib
{
    namespace Engine
    {
        class ITMIMUOnlyTracker: public ITMTracker // derive from ITMIMUTracker instead?
        {
        private:
            ITMIMUCalibrator *calibrator;

        public:
            void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

            ITMIMUOnlyTracker(ITMLib::Objects::ITMIMUCalibrator* calibrator);
            virtual ~ITMIMUOnlyTracker(void);
        };
    }
}



