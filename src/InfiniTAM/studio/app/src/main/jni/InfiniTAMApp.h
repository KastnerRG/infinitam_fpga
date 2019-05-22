// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include <stdlib.h>

#include "../../../../../Engine/IMUSourceEngine.h"
#include "../../../../../Engine/ImageSourceEngine.h"
#include "../../../../../ITMLib/ITMLib.h"
#include "../../../../../Utils/NVTimer.h"

#include "TouchHandler.h"
#include "DataBuffer.h"

#include <mutex>

class InfiniTAMApp {
public:
	static InfiniTAMApp* Instance(void)
	{
		if (globalInstance==NULL) globalInstance = new InfiniTAMApp();
		return globalInstance;
	}

	InfiniTAMApp(void);
	~InfiniTAMApp(void);

	void InitGL();
	void ResizeGL(int newWidth, int newHeight);
	void RenderGL(void);

	void StartProcessing(int useLiveCamera, float voxelSize = 0.005f, int icp_quality = 4);
	bool ProcessFrame(void);

	bool IsInitialized(void) const
	{ return mIsInitialized; }

	void ResetScene();
	void SetIntegration(bool on = true);
	void SetMainProcessing(bool on = true);
	void SaveScene(bool mesh = false);
	void Stop();

	TouchHandler& GetTouchHandler(){ return mTouchHandler; }
	void ResetView();

private:
	static float normal_pdf(float x, float m, float s);

private:

	static InfiniTAMApp *globalInstance;

	InfiniTAM::Engine::ImageSourceEngine *mImageSource;
	InfiniTAM::Engine::IMUSourceEngine *mImuSource;
	ITMLib::Objects::ITMLibSettings *mInternalSettings;
	ITMLib::Engine::ITMMainEngine *mMainEngine;

	ITMUChar4Image *inputRGBImage; ITMShortImage *inputRawDepthImage;
	ITMIMUMeasurement *inputIMUMeasurement;

	StopWatchInterface *timer_instant;
	StopWatchInterface *timer_average;
	StopWatchInterface *timer_view;
	StopWatchInterface *timer_frame;

	static const int NUM_WIN = 3;
	Vector4f winPos[NUM_WIN];
	uint textureId[NUM_WIN];
	ITMMainEngine::GetImageType winImageType[NUM_WIN];
	ITMUChar4Image *outImage[NUM_WIN];

	Vector2i mNewWindowSize;
	bool mIsInitialized;

	DataBuffer< std::pair<ITMPose, ITMIMUMeasurement> > mCurrentPoseBuffer;
	bool mNewPose;
	int POSE_COUNT_MAX;
	int mPoseCount;
	bool mIsStopped;

	TouchHandler mTouchHandler;
};


