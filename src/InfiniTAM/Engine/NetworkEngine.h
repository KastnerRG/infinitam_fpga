/*
 * NetworkEngine.h
 *
 *  Created on: Feb 9, 2017
 *      Author: qkgautier
 */

#pragma once

#include "ImageSourceEngine.h"
#include "IMUSourceEngine.h"

#include "../Utils/UdpClient.h"

namespace InfiniTAM
{
	namespace Engine
	{
		/**
		 * Note: Mostly copied from CLIEngine, but the Singleton makes it hard to derive the class.
		 */
		class NetworkEngine
		{
		private:
			static NetworkEngine* instance;

			ITMLibSettings internalSettings;
			ImageSourceEngine *imageSource;
			IMUSourceEngine *imuSource;
			ITMMainEngine *mainEngine;

			StopWatchInterface *timer_instant;
			StopWatchInterface *timer_average;

			ITMUChar4Image *inputRGBImage;
			ITMShortImage *inputRawDepthImage;
			ITMIMUMeasurement *inputIMUMeasurement;

			int currentFrameNo;


			ITMUChar4Image* image_;
			UdpClient* client_;
			boost::asio::io_service io_service_;

		public:
			static NetworkEngine* Instance(void)
			{
				if (instance == NULL) instance = new NetworkEngine();
				return instance;
			}

			float processedTime;

			void Initialise(
					ImageSourceEngine *imageSource,
					IMUSourceEngine *imuSource,
					ITMMainEngine *mainEngine,
					ITMLibSettings::DeviceType deviceType,
					const std::string& host,
					const std::string& port);

			void Shutdown();

			void Run();
			bool ProcessFrame();
		};
	}
}

