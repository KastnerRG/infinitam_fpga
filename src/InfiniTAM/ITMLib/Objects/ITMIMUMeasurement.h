// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

namespace ITMLib
{
	namespace Objects
	{
		class ITMIMUMeasurement
		{
		public:
			Matrix3f R;
			Vector3f t;

			ITMIMUMeasurement()
			{
				this->R.setIdentity();
				this->t = Vector3f(0.f);
			}

			ITMIMUMeasurement(const Matrix3f & R)
			{
				this->R = R;
				this->t = Vector3f(0.f);
			}

			ITMIMUMeasurement(const Matrix3f & R, const Vector3f& t)
			{
				this->R = R;
				this->t = t;
			}

			void SetFrom(const ITMIMUMeasurement *measurement)
			{
				this->R = measurement->R;
				this->t = measurement->t;
			}

			~ITMIMUMeasurement(void) { }

			// Suppress the default copy constructor and assignment operator // why??

			// Unsuppressing
			ITMIMUMeasurement(const ITMIMUMeasurement& other){ this->SetFrom(&other); }
			ITMIMUMeasurement& operator=(const ITMIMUMeasurement& other)
			{
				if(&other == this)
					return *this;
				this->SetFrom(&other);
				return *this;
			}
		};
	}
}
