// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib/ITMLib.h"

namespace InfiniTAM
{
	namespace Engine
	{
		class IMUSourceEngine
		{
		protected:
			static const int BUF_SIZE = 2048;
			char imuMask[BUF_SIZE];

			ITMIMUMeasurement *cached_imu;

			virtual void loadIMUIntoCache();

			int cachedFrameNo;
			int currentFrameNo;

		public:
			IMUSourceEngine(const char *imuMask);
			virtual ~IMUSourceEngine() { }

			virtual bool hasMoreMeasurements(void);
			virtual void getMeasurement(ITMIMUMeasurement *imu);
		};
	}
}

