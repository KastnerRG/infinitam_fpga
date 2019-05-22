/*
 * NetworkEngine.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: qkgautier
 */

#include "NetworkEngine.h"

#include <cstring>
#include <stdint.h>

#include "../Utils/FileUtils.h"

using namespace InfiniTAM::Engine;
using namespace std;

NetworkEngine* NetworkEngine::instance;

void NetworkEngine::Initialise(
		ImageSourceEngine *imageSource,
		IMUSourceEngine *imuSource,
		ITMMainEngine *mainEngine,
		ITMLibSettings::DeviceType deviceType,
		const string& host,
		const string& port)
{
	this->imageSource = imageSource;
	this->imuSource   = imuSource;
	this->mainEngine  = mainEngine;

	this->currentFrameNo = 0;

	bool allocateGPU = false;
	if (deviceType == ITMLibSettings::DEVICE_CUDA) allocateGPU = true;

	inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, allocateGPU);
	inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, allocateGPU);
	inputIMUMeasurement = new ITMIMUMeasurement();

	client_ = new UdpClient(io_service_, host, port);

	image_ = new ITMUChar4Image(imageSource->getDepthImageSize(), true, allocateGPU);


#ifndef COMPILE_WITHOUT_CUDA
	ITMSafeCall(cudaThreadSynchronize());
#endif

	sdkCreateTimer(&timer_instant);
	sdkCreateTimer(&timer_average);

	sdkResetTimer(&timer_average);

	printf("initialised.\n");
}

bool NetworkEngine::ProcessFrame()
{
	if (!imageSource->hasMoreImages()) return false;
	imageSource->getImages(inputRGBImage, inputRawDepthImage);

	if (imuSource != NULL) {
		if (!imuSource->hasMoreMeasurements()) return false;
		else imuSource->getMeasurement(inputIMUMeasurement);
	}

	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant); sdkStartTimer(&timer_average);

	//actual processing on the mailEngine
	if (imuSource != NULL) mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
	else mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);

#ifndef COMPILE_WITHOUT_CUDA
	ITMSafeCall(cudaThreadSynchronize());
#endif
	sdkStopTimer(&timer_instant); sdkStopTimer(&timer_average);

	float processedTime_inst = sdkGetTimerValue(&timer_instant);
	float processedTime_avg = sdkGetAverageTimerValue(&timer_average);

	printf("Frame %i - Total_time: %.2fms (Average: %.2fms)\n", currentFrameNo, processedTime_inst, processedTime_avg);


	// Send image using UDP:
	//     - Divide image into chunks of 1024 bytes
	//     - Send [chunk index, chunk data] = 1032 bytes
	// (Outside of timers?)
	// TODO in different thread
	{
		mainEngine->GetImage(image_, ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST);


		size_t total_size = image_->dataSize * sizeof(Vector4u);
		size_t step_size  = 1024;
		size_t idx_size   = sizeof(uint64_t);

		const char* image_data = (const char*)image_->GetData(MEMORYDEVICE_CPU);

		client_->sendLargeBuffer(image_data, total_size, (uint32_t)currentFrameNo, step_size);
	}


	currentFrameNo++;

	return true;
}

void NetworkEngine::Run()
{
	while (true) {
		if (!ProcessFrame()) break;
	}
}

void NetworkEngine::Shutdown()
{
	sdkDeleteTimer(&timer_instant);
	sdkDeleteTimer(&timer_average);

	delete inputRGBImage;
	delete inputRawDepthImage;
	delete inputIMUMeasurement;
	delete client_;
	delete image_;

	delete instance;
}
