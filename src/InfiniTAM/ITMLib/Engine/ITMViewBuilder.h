// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Utils/ITMLibSettings.h"

#include "../Objects/ITMView.h"
#include "../Objects/ITMViewIMU.h"
#include "../Objects/ITMRGBDCalib.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		*/
		class ITMViewBuilder
		{
		protected:
			const ITMRGBDCalib *calib;
			ITMShortImage *shortImage;
			ITMFloatImage *floatImage;

		public:
			virtual void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics,
				Vector2f disparityCalibParams) = 0;
			virtual void ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams) = 0;

			virtual void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in) = 0;
			virtual void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic) = 0;

			virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise = false) = 0;
			virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage) = 0;

			virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter,
				ITMIMUMeasurement *imuMeasurement) = 0;

			ITMViewBuilder(const ITMRGBDCalib *calib)
			{
				this->calib = calib;
				this->shortImage = NULL;
				this->floatImage = NULL;
			}

			virtual ~ITMViewBuilder()
			{
				if (this->shortImage != NULL) delete this->shortImage;
				if (this->floatImage != NULL) delete this->floatImage;
			}
		};
	}
}

