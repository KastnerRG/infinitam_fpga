//
// Created by qkgautier on 8/7/16.
//

#pragma once

#include "../../../../../ITMLib/ITMLib.h"

#include <mutex>


class TouchHandler
{
public:
    TouchHandler(const ITMPose* initialPose = NULL);

    void ResetPose(const ITMPose* initialPose = NULL);

    void SetScreenSize(float x, float y);

    void onTouchEvent(int touch_count, int event, float x0, float y0, float x1, float y1);

    ITMPose GetPose();

private:

    static const int ACTION_DOWN         = 0;
    static const int ACTION_UP           = 1;
    static const int ACTION_MOVE         = 2;
    static const int ACTION_CANCEL       = 3;
    static const int ACTION_POINTER_DOWN = 5;

    Vector2f screenSize_;

    ITMPose pose_;
    Vector2f orig_0_;
    Vector2f orig_1_;

    std::mutex pose_mutex_;
};


