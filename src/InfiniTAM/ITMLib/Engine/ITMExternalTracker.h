//
// Created by qkgautier on 5/4/17.
//

#pragma once

#include "../Objects/ITMIMUMeasurement.h"
#include "../Engine/ITMTracker.h"


namespace ITMLib
{
    namespace Engine
    {
        class ITMExternalTracker: public ITMTracker
        {
        private:

        public:
            void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

            ITMExternalTracker();
            virtual ~ITMExternalTracker(void);
        };
    }
}
