// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "InfiniTAMApp.h"
#include "../../../../../Engine/OpenNIEngine.h"
#include "../../../../../Engine/IMUSourceEngine.h"
#include "../../../../../Engine/TangoEngine.h"
#include "../../../../../ITMLib/Engine/ITMIMUCalibrator_Tango.h"
#include "../../../../../ORUtils/Quaternion.h"

#include "../../../../../ITMLib/Engine/DeviceSpecific/CUDA/CudaUtils.h"

#include <GLES/gl.h>

#include <android/log.h>
#include <unistd.h>

#include <chrono>
#include <sstream>
#include <iomanip>
#include <cmath>

#include "logging.h"
#include "JavaMethods.h"


InfiniTAMApp* InfiniTAMApp::globalInstance = NULL;

InfiniTAMApp::InfiniTAMApp(void)
{
	mImageSource = NULL;
	mImuSource = NULL;
	mMainEngine = NULL;
	mIsInitialized = false;
	mIsStopped = false;

	if(ITMVoxel::hasColorInformation)
	{ winImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME; }
	else
	{ winImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED; }
	winImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
	winImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;
}

void InfiniTAMApp::InitGL(void)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glGenTextures(NUM_WIN, textureId);
}

void InfiniTAMApp::ResizeGL(int newWidth, int newHeight)
{
	mNewWindowSize.x = newWidth;
	mNewWindowSize.y = newHeight;
}

void InfiniTAMApp::RenderGL(void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	if (!IsInitialized()) return;

	if (mNewWindowSize.x > 0) {
		Vector2i depthImageSize = mMainEngine->GetImageSize();
		float ratio = (float)depthImageSize.x/(float)depthImageSize.y;

//		glViewport(0, 0, newWidth, newHeight);
		if (mNewWindowSize.x >= mNewWindowSize.y) {

#ifdef COMPILE_FOR_TANGO
			winPos[0] = Vector4f(0.0f, 0.0f, 1.0f, 1.0f);
			winPos[1] = Vector4f(0.8f, 0.8f, 1.0f, 1.0f);
			winPos[2] = Vector4f(0.8f, 0.6f, 1.0f, 0.8f);
#else
			winPos[0] = Vector4f(0.0f, 0.0f, 1.0f/ratio, 1.0f);
			winPos[1] = Vector4f(1.0f/ratio, 0.5f, 1.0f, 1.0f);
			winPos[2] = Vector4f(1.0f/ratio, 0.0f, 1.0f, 0.5f);
#endif

		} else {
			winPos[0] = Vector4f(0.0f, 1.0f/3.0f, 1.0f, 1.0f);
			winPos[1] = Vector4f(0.0f, 0.0f, 0.5f, 1.0f/3.0f);
			winPos[2] = Vector4f(0.5f, 0.0f, 1.0f, 1.0f/3.0f);
		}

		mNewWindowSize.x = mNewWindowSize.y = -1;
	}

	ITMPose pose;


	// Use IMU pose to interpolate
	//
#ifdef COMPILE_FOR_TANGO
	InfiniTAM::Engine::TangoEngine* tango_engine = dynamic_cast<InfiniTAM::Engine::TangoEngine*>(mImageSource);
	if(tango_engine)
//	if(false)
	{
//		ITMIMUCalibrator_Tango mImuCalibrator;
//
//		mImuCalibrator.RegisterMeasurement(imuPose_ref.R);
//		mImuCalibrator.RegisterMeasurement(imuPose_ref.t);

        // Tango current pose
		ITMIMUMeasurement imuPose;
		tango_engine->getCurrentMeasurement(&imuPose);
//		mImuCalibrator.RegisterMeasurement(imuPose.R);
//		mImuCalibrator.RegisterMeasurement(imuPose.t);

//		pose.SetR(mImuCalibrator.GetDifferentialRotationChange() * pose.GetR());
//		pose.SetT(mImuCalibrator.GetDifferentialTranslationChange() + pose.GetT());



		const int NUM_POSES_INTERPOLATION = std::max(1, std::min(10, mMainEngine->GetNumFrames()));



        // Get several tango reference poses

        std::vector< std::pair<ITMPose, ITMIMUMeasurement> > tango_ref_poses;

        int num_ref_poses = 1; //std::min(NUM_POSES_INTERPOLATION, mMainEngine->GetNumFrames()+1);
        mCurrentPoseBuffer.getMultipleData(0, num_ref_poses-1, tango_ref_poses);



        // Reset pose counter if pose has changed.
        //
        // Update max pose count to reflect the time between two new poses.
        // (We expect this time to be roughly the same for one implementation
        //  but we don't necessarily know it beforehand)
        if(mNewPose)
        {
			mNewPose = false;
            POSE_COUNT_MAX = std::max(1, (POSE_COUNT_MAX + (mPoseCount-1)) / 2);
			mPoseCount = 0;
        }

//		POSE_COUNT_MAX = 5;
//		LOGI("POSE_COUNT_MAX: %d", POSE_COUNT_MAX);


        // Calculate a new pose based on each reference pose and the current one
		std::vector<ITMPose> kinfuPoses(num_ref_poses);
        for(int i = 0; i < num_ref_poses; i++)
        {
			ITMIMUCalibrator_Tango mImuCalibrator;

			// Tango reference pose
			mImuCalibrator.RegisterMeasurement(tango_ref_poses[i].second.R);
			mImuCalibrator.RegisterMeasurement(tango_ref_poses[i].second.t);

			// Tango current pose
			mImuCalibrator.RegisterMeasurement(imuPose.R);
			mImuCalibrator.RegisterMeasurement(imuPose.t);

			ITMPose newpose;

			newpose.SetRT(mImuCalibrator.GetDifferentialRotationChange(), mImuCalibrator.GetDifferentialTranslationChange());

			kinfuPoses[i].SetM(tango_ref_poses[i].first.GetM() * newpose.GetM());
        }

        // Now we have a set of similar poses,
        // we will average them with different weights.

        // Set a weight for each pose based on a distribution curve.
        // The pose counter will shift this curve incrementally.
		std::vector<float> weights(num_ref_poses);
        float bounded_pose_count = std::min(mPoseCount, POSE_COUNT_MAX);
        for(int i = 0; i < num_ref_poses; i++)
        {
            int center = num_ref_poses / 2;
            weights[i] = normal_pdf(i,
                                    center - float(bounded_pose_count)/POSE_COUNT_MAX,
                                    sqrtf(float(num_ref_poses - center) / 3.f));
        }



        // Now we interpolate rotation and translation
		Matrix3f pose_r0 = kinfuPoses[0].GetR();
		Vector4f q_interp = ORUtils::QuaternionFromRotationMatrix(pose_r0);
        Vector3f t_interp = kinfuPoses[0].GetT();
        float weight_sum = weights[0];
        for(int i = 1; i < num_ref_poses; i++)
        {
			Matrix3f pose_r1 = kinfuPoses[i].GetR();
			Vector4f q1 = ORUtils::QuaternionFromRotationMatrix(pose_r1);

            Vector3f t1 = kinfuPoses[i].GetT();

            weight_sum += weights[i];
            float w = weights[i] / weight_sum;

			q_interp = ORUtils::QuaternionLerp(q_interp, q1, w); // double check

            t_interp.x = t_interp.x * (1.f-w) + t1.x * w;
            t_interp.y = t_interp.y * (1.f-w) + t1.y * w;
            t_interp.z = t_interp.z * (1.f-w) + t1.z * w;
        }


		Matrix3f r_interp = ORUtils::QuaternionToRotationMatrix(q_interp);
		pose.SetRT(r_interp, t_interp);


        // Increase pose counter
        if(true)
        { mPoseCount++; }


//		//debug
//		pose.SetRT(imuPose.R, imuPose.t);

	}
	else
#endif
	{
		std::pair<ITMPose, ITMIMUMeasurement> data = mCurrentPoseBuffer.getData();
		pose.SetFrom(&data.first);
	}

	ITMPose deltaPose = mTouchHandler.GetPose();
	ITMPose delta1(0.f, 0.f, -0.5f, 0.f, 0.f, 0.f);
	ITMPose delta1inv(0.f, 0.f, 0.5f, 0.f, 0.f, 0.f);
	delta1.MultiplyWith(&pose);
	deltaPose.MultiplyWith(&delta1);
	delta1inv.MultiplyWith(&deltaPose);
	pose.SetFrom(&delta1inv);
//	pose.MultiplyWith(&deltaPose);

	int localNumWin = 2;//NUM_WIN
	for (int w = 0; w < localNumWin; w++)
	{
		mMainEngine->GetImage(outImage[w], winImageType[w],
							  &pose, &mImageSource->calib.intrinsics_d);
	}

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	{
		glLoadIdentity();
		glOrthof(0.0f, 1.0f, 0.0f, 1.0f, -1.0f, 1.0f);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		{
			glEnable(GL_TEXTURE_2D);
			glDisable(GL_BLEND);
			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
			for (int w = 0; w < localNumWin; w++) {
				glBindTexture(GL_TEXTURE_2D, textureId[w]);
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, outImage[w]->noDims.x, outImage[w]->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, outImage[w]->GetData(MEMORYDEVICE_CPU));
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

				float vertices[] = {
					winPos[w][0], winPos[w][3],
					winPos[w][2], winPos[w][3],
					winPos[w][0], winPos[w][1],
					winPos[w][2], winPos[w][1] };
				float texture[] = {
					0.0f, 0.0f,
					1.0f, 0.0f,
					0.0f, 1.0f,
					1.0f, 1.0f };

				glVertexPointer(2, GL_FLOAT, 0, vertices);
				glTexCoordPointer(2, GL_FLOAT, 0, texture);
				glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
			}
			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_TEXTURE_COORD_ARRAY);
			glDisable(GL_TEXTURE_2D);
		}
		glPopMatrix();
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();


	sdkStopTimer(&timer_view);

	//LOGI("Display: %f FPS", 1000.0/sdkGetTimerValue(&timer_view));

	sdkResetTimer(&timer_view);
	sdkStartTimer(&timer_view);
}

void InfiniTAMApp::StartProcessing(int useLiveCamera, float voxelSize, int icp_quality)
{
	const char *calibFile = "/storage/sdcard0/InfiniTAM/Teddy/calib.txt";
	const char *imagesource_part1 = "/storage/sdcard0/InfiniTAM/Teddy/Frames/%04i.ppm";
	const char *imagesource_part2 = "/storage/sdcard0/InfiniTAM/Teddy/Frames/%04i.pgm";

	mInternalSettings = new ITMLibSettings(ITMLibSettings::TRACKER_IMU_INIT);
	mInternalSettings->sceneParams.voxelSize = voxelSize;
	mInternalSettings->sceneParams.viewFrustum_min = 0.4f;
	mInternalSettings->sceneParams.viewFrustum_max = 5.0f;
	mInternalSettings->useSwapping = false;
	mInternalSettings->deviceType = ITMLibSettings::DEVICE_CUDA;
	for(int i = 0; i < mInternalSettings->noHierarchyLevels; i++)
	{
		mInternalSettings->noICPIterations[i] = icp_quality + icp_quality*i;
	}
	mInternalSettings->depthTrackerTerminationThreshold = 0.00001f;

	mImuSource = NULL; //new IMUSourceEngine;
	if (useLiveCamera == 0) {
//		mImageSource = new InfiniTAM::Engine::ImageFileReader(calibFile, imagesource_part1, imagesource_part2);
		//mImageSource = new InfiniTAM::Engine::RawFileReader(calibFile, imagesource_part1, imagesource_part2, Vector2i(320, 240), 0.5f);
		//mImuSource = new InfiniTAM::Engine::IMUSourceEngine(imagesource_part3);
		//mImageSource = new InfiniTAM::Engine::OpenNIEngine(calibFile, "/storage/sdcard0/InfiniTAM/50Hz_closeup.oni");
		mImageSource = InfiniTAM::Engine::TangoEngine::getInstance();

		// Have to do this because of asynchronous Tango initialization. I'm sure it could be done better.
		InfiniTAM::Engine::TangoEngine* tango_engine = dynamic_cast<InfiniTAM::Engine::TangoEngine*>(mImageSource);
		if(tango_engine) { while(!tango_engine->isInitialized()){ usleep(10000); } }

	} else {
		mImageSource = new InfiniTAM::Engine::OpenNIEngine(calibFile);
	}
	mMainEngine = new ITMMainEngine(mInternalSettings, &mImageSource->calib, mImageSource->getRGBImageSize(), mImageSource->getDepthImageSize());


	bool allocateGPU = false;
	if (mInternalSettings->deviceType == ITMLibSettings::DEVICE_CUDA) allocateGPU = true;

	for (int w = 0; w < NUM_WIN; w++) {
		outImage[w] = new ITMUChar4Image(mImageSource->getDepthImageSize(), true, allocateGPU);
	}

	inputRGBImage = new ITMUChar4Image(mImageSource->getRGBImageSize(), true, allocateGPU);
	inputRawDepthImage = new ITMShortImage(mImageSource->getDepthImageSize(), true, allocateGPU);
	inputIMUMeasurement = new ITMIMUMeasurement();

#ifndef COMPILE_WITHOUT_CUDA
	ITMSafeCall(cudaThreadSynchronize());
#endif

	sdkCreateTimer(&timer_instant);
	sdkCreateTimer(&timer_average);
	sdkCreateTimer(&timer_view);
	sdkResetTimer(&timer_average);
	sdkCreateTimer(&timer_frame);

	mNewPose = false;
	POSE_COUNT_MAX = 10;
	mPoseCount = 0;
	ResetView();

	mIsInitialized = true;
}

bool InfiniTAMApp::ProcessFrame(void)
{
	if (!mImageSource->hasMoreImages() || mIsStopped) return false;

#ifdef COMPILE_FOR_TANGO
	InfiniTAM::Engine::TangoEngine* tango_engine = dynamic_cast<InfiniTAM::Engine::TangoEngine*>(mImageSource);
	if(tango_engine)
	{
		if(!tango_engine->isImageReady()){ return true; }
	}
#endif

	mImageSource->getImages(inputRGBImage, inputRawDepthImage);


//	SaveImageToFile(inputRawDepthImage, "/sdcard/test.pnm");
//	SaveImageToFile(inputRGBImage, "/sdcard/test_rgb.pnm");

	if (mImuSource != NULL) {
		if (!mImuSource->hasMoreMeasurements()) return false;
		else mImuSource->getMeasurement(inputIMUMeasurement);
	}
#ifdef COMPILE_FOR_TANGO
	else if(tango_engine)
	{
		bool imugood = tango_engine->getMeasurement(inputIMUMeasurement, mMainEngine->GetNumFrames() == 0);
		if(!imugood){ return true; }
	}
#endif

	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant); sdkStartTimer(&timer_average);


	bool hasIMU = (mImuSource != NULL);

#ifdef COMPILE_FOR_TANGO
	if(tango_engine){ hasIMU = true; }
#endif


	//actual processing on the mainEngine
	if (hasIMU)
	{
		mMainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
	}
	else
	{
		mMainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);
	}


	ITMSafeCall(cudaDeviceSynchronize());
	sdkStopTimer(&timer_instant); sdkStopTimer(&timer_average);

	//__android_log_print(ANDROID_LOG_VERBOSE, "InfiniTAM", "Process Frame finished: %f %f", sdkGetTimerValue(&timer_instant), sdkGetAverageTimerValue(&timer_average));

	sdkStopTimer(&timer_frame);


	if(mMainEngine->GetNumFrames() % 10 == 0)
	{
		float time_frame = sdkGetAverageTimerValue(&timer_frame);
		float time_algo = sdkGetAverageTimerValue(&timer_average);
		JavaMethods::GetInstance().setAlgorithmFramerate(1000.f / time_algo);
		sdkResetTimer(&timer_frame);
		sdkResetTimer(&timer_average);
	}

	sdkStartTimer(&timer_frame);

	// Update pose
//	if(mMainEngine->GetNumFrames() <= 1)
	{
		mCurrentPoseBuffer.addData(std::make_pair(*mMainEngine->GetTrackingState()->pose_d, *inputIMUMeasurement));
		mNewPose = true;
	}


#ifdef COMPILE_FOR_TANGO
	if(tango_engine){ tango_engine->setLastCalculatedPose(*mMainEngine->GetTrackingState()->pose_d); }
#endif


	// Query memory usage (not accurate)
//#ifndef COMPILE_WITHOUT_CUDA
//	size_t free_byte, total_byte;
//	getCudaMemoryUsage(free_byte, total_byte);
//
//	JavaMethods::GetInstance().setMemoryUsage(free_byte, total_byte);
//#endif

	return true;
}

void InfiniTAMApp::ResetScene()
{
	if(mIsInitialized)
	{
		mMainEngine->ResetScene();
		ResetView();
	}
}


void InfiniTAMApp::SetIntegration(bool on)
{
	if(mIsInitialized)
	{
		on? mMainEngine->turnOnIntegration(): mMainEngine->turnOffIntegration();
	}
}

void InfiniTAMApp::SetMainProcessing(bool on)
{
	if(mIsInitialized)
	{
		on? mMainEngine->turnOnMainProcessing(): mMainEngine->turnOffMainProcessing();
	}
}

void InfiniTAMApp::SaveScene(bool mesh)
{
	if(mIsInitialized)
	{
		std::string time_filename;

		auto now = std::chrono::system_clock::now();
		auto now_c = std::chrono::system_clock::to_time_t(now);

		char buffer_time[100];
		strftime(buffer_time, sizeof(buffer_time), "%Y_%m_%d_%H_%M_%S", std::localtime(&now_c));

		std::stringstream ss;
		ss << "/sdcard/InfiniTAM";
		ss << "/world_";
		ss << buffer_time;
		ss << "_" << mInternalSettings->sceneParams.voxelSize;
		if(mesh){ ss << ".stl"; }
		else{ ss << ".pcd"; }
		time_filename = ss.str();

//		std::string filename;
//		if(mesh){ filename = "/sdcard/InfiniTAM/world.stl"; }
//		else{ filename = "/sdcard/InfiniTAM/world.pcd"; }



		if(mesh)
		{
			mMainEngine->SaveSceneToMesh(time_filename.c_str());
//			mMainEngine->SaveSceneToMesh(filename.c_str());
		}
		else
		{
			mMainEngine->SaveTSDFToFile(time_filename.c_str());
//			mMainEngine->SaveTSDFToFile(filename.c_str());
		}

		LOGI("Mesh save to \"%s\"", time_filename.c_str());
//		LOGI("Mesh save to \"%s\"", filename.c_str());
	}
}

void InfiniTAMApp::ResetView()
{
	ITMPose deltaPose; //(0.f, 0.f, 0.2f, 0.f, 0.f, 0.f);
	mTouchHandler.ResetPose(&deltaPose);
}


void InfiniTAMApp::Stop()
{
#ifdef COMPILE_FOR_TANGO
	InfiniTAM::Engine::TangoEngine* tango_engine = dynamic_cast<InfiniTAM::Engine::TangoEngine*>(mImageSource);
	if(tango_engine)
	{
		tango_engine->stopRecording();
	}
#endif
	mIsStopped = true;
}

float InfiniTAMApp::normal_pdf(float x, float m, float s)
{
	static const float inv_sqrt_2pi = 0.3989422804014327;
	float a = (x - m) / s;

	return inv_sqrt_2pi / s * std::exp(-0.5f * a * a);
}




