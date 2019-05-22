//
// Created by qkgautier on 7/31/16.
//

#include "ITMIMUOnlyTracker.h"

#include "../Objects/ITMViewIMU.h"

#ifdef COMPILE_FOR_TANGO
#include "../../Engine/logging.h" // for debug
#endif


using namespace ITMLib::Engine;

ITMIMUOnlyTracker::ITMIMUOnlyTracker(ITMIMUCalibrator *calibrator)
{
    this->calibrator = calibrator;
}

ITMIMUOnlyTracker::~ITMIMUOnlyTracker(void)
{
}

void ITMIMUOnlyTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
    calibrator->RegisterMeasurement(((ITMLib::Objects::ITMViewIMU*)view)->imu->R);
    calibrator->RegisterMeasurement(((ITMLib::Objects::ITMViewIMU*)view)->imu->t);

    ITMPose Pdiff;
    Pdiff.SetRT(calibrator->GetDifferentialRotationChange(), calibrator->GetDifferentialTranslationChange());

    trackingState->pose_d->MultiplyWith(&Pdiff);
//    *trackingState->pose_d = Pdiff;
}