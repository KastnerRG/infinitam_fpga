//
// Created by qkgautier on 8/7/16.
//

#include "TouchHandler.h"


TouchHandler::TouchHandler(const ITMPose* initialPose):
        orig_0_(0.f, 0.f),
        orig_1_(0.f, 0.f)
{
    ResetPose(initialPose);
}

void TouchHandler::ResetPose(const ITMPose *initialPose)
{
    std::lock_guard<std::mutex> guard(pose_mutex_);
    if(initialPose){ pose_.SetFrom(initialPose); }
    else{ pose_.SetFrom(0.f, 0.f, 0.f, 0.f, 0.f, 0.f); }
}

void TouchHandler::SetScreenSize(float x, float y)
{
    screenSize_.x = x;
    screenSize_.y = y;
}

void TouchHandler::onTouchEvent(int touch_count, int event,
                                float x0, float y0,
                                float x1, float y1)
{
    float minSize = std::min(screenSize_.x, screenSize_.y);

    x0 = (x0 * screenSize_.x) / minSize;
    x1 = (x1 * screenSize_.x) / minSize;
    y0 = (y0 * screenSize_.y) / minSize;
    y1 = (y1 * screenSize_.y) / minSize;


    Vector2f offset_0, offset_1;

    switch(event)
    {
        case ACTION_DOWN:
        case ACTION_POINTER_DOWN:
            orig_0_ = Vector2f(x0, y0);
            orig_1_ = Vector2f(x1, y1);
            return;

        case ACTION_MOVE:
            offset_0 = Vector2f(x0, y0) - orig_0_;
            offset_1 = Vector2f(x1, y1) - orig_1_;
            break;

        default:
            return;
    }

    std::lock_guard<std::mutex> guard(pose_mutex_);
    if(touch_count == 1)
    {
        offset_0 /= 1.5f;
        ITMPose rot(0.f, 0.f, 0.f, -offset_0.y, offset_0.x, 0.f);
        pose_.SetM(rot.GetM() * pose_.GetM());
    }
    else if(touch_count == 2)
    {
        float eucl_orig = sqrtf((orig_0_.x-orig_1_.x)*(orig_0_.x-orig_1_.x) + (orig_0_.y-orig_1_.y)*(orig_0_.y-orig_1_.y));
        float eucl_curr = sqrtf((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));

        ITMPose trans(0.f, 0.f, (eucl_orig-eucl_curr) * 2.f, 0.f, 0.f, 0.f);
        pose_.SetM(trans.GetM() * pose_.GetM());
    }

    orig_0_ = Vector2f(x0, y0);
    orig_1_ = Vector2f(x1, y1);
}


ITMPose TouchHandler::GetPose()
{
    std::lock_guard<std::mutex> guard(pose_mutex_);
    return pose_;
}









