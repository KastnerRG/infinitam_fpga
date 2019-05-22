//
// Created by qkgautier on 7/24/16.
//

#pragma once

#include "ImageSourceEngine.h"
#include "IMUSourceEngine.h"
#include "../Utils/TangoImages.h"
#include "../Utils/DataWriter.h"

#include "tango_client_api.h"
#include "tango_support_api.h"

#include <memory>
#include <atomic>


namespace InfiniTAM
{
    namespace Engine
    {
        class TangoEngine : public ImageSourceEngine
        {
        public:
            static TangoEngine* getInstance();

            bool initialize(int32_t max_point_cloud_elements);
            int connectCallbacks();

            bool hasMoreImages(void);
            void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
            Vector2i getDepthImageSize(void);
            Vector2i getRGBImageSize(void);

            bool isImageReady();
            bool isInitialized(){ return is_initialized_; }

            bool getMeasurement(ITMIMUMeasurement *imu, bool first = false);
            bool getCurrentMeasurement(ITMIMUMeasurement *imu, bool first = false);

            void setLastCalculatedPose(const ITMPose& pose);
            void stopRecording();

            ~TangoEngine();

        private:
            TangoEngine(const char *calibFilename);

            static void onPointCloudAvailable(void* context, const TangoPointCloud* point_cloud);
            static void onColorFrameAvailable(void *context, TangoCameraId id, const TangoImageBuffer *buffer);
            static void onFisheyeFrameAvailable(void *context, TangoCameraId id, const TangoImageBuffer *buffer);
            static void onPoseAvailable(void*, const TangoPoseData* pose);

            void getRgb(double timestamp, ITMUChar4Image *rgb);
            void convertDepth(TangoPointCloud* point_cloud, ITMShortImage *rawDepth);
            bool getMeasurementAtTimestamp(ITMIMUMeasurement *imu, double timestamp, bool first = false);

        private:
            static std::unique_ptr<TangoEngine> tango_engine_;

            std::atomic<bool> is_initialized_;

            int32_t max_point_cloud_elements_;
            bool pose_valid_;

            TangoSupportPointCloudManager* pc_manager_;
//            TangoSupportDepthInterpolator *depth_interpolator_;
            double last_depth_timestamp_;

            TangoImages color_image_buffer_;
            TangoImages fisheye_buffer_;
            DataWriter data_writer_; ///< Class to save data to files

            TangoCameraIntrinsics depthIntr_;
            TangoCameraIntrinsics ccIntrinsics_;
            TangoCameraIntrinsics fisheyeIntrinsics_;


            ORUtils::Matrix4<double> first_imu_pose_inv_;
        };
	}
}








