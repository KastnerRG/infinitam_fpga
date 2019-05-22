//
// Created by qkgautier on 5/4/17.
//

#include "ITMExternalTracker.h"

#include "../Objects/ITMViewIMU.h"



using namespace ITMLib::Engine;

ITMExternalTracker::ITMExternalTracker()
{
}

ITMExternalTracker::~ITMExternalTracker(void)
{
}

void ITMExternalTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
    trackingState->pose_d->SetRT(((ITMLib::Objects::ITMViewIMU*)view)->imu->R,
                                 ((ITMLib::Objects::ITMViewIMU*)view)->imu->t);
}
