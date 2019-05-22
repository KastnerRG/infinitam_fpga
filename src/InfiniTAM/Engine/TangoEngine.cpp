//
// Created by qkgautier on 7/24/16.
//

#include "TangoEngine.h"
#include "../ORUtils/Quaternion.h"

#include "logging.h"

#include <limits>

using namespace InfiniTAM::Engine;
using namespace std;



std::unique_ptr<TangoEngine> TangoEngine::tango_engine_;



//    static void tangoPoseToAffine3f(const TangoPoseData& tango_pose, Eigen::Affine3f& affine)
//    {
//        Eigen::Quaternion <float> q(tango_pose.orientation[3], tango_pose.orientation[0],
//                                    tango_pose.orientation[1], tango_pose.orientation[2]);
//        Eigen::Translation <float, 3> t(Eigen::Vector3f(tango_pose.translation[0],
//                                                        tango_pose.translation[1],
//                                                        tango_pose.translation[2]));
//        affine = Eigen::Affine3f( Eigen::Translation3f(t) * q);
//    }



void TangoEngine::onPointCloudAvailable(void* context, const TangoPointCloud* point_cloud)
{
    if(!tango_engine_->pose_valid_){ return; }

    TangoSupport_updatePointCloud(tango_engine_->pc_manager_, point_cloud);
}

void TangoEngine::onColorFrameAvailable(void *context, TangoCameraId id, const TangoImageBuffer *buffer)
{
//    tango_handler.data_writer_.addImageToSave(buffer->data, buffer->timestamp);

    tango_engine_->color_image_buffer_.addNewData(buffer);
}

void TangoEngine::onFisheyeFrameAvailable(void *context, TangoCameraId id, const TangoImageBuffer *buffer)
{
    if(tango_engine_->data_writer_.isRecording())
    {
        tango_engine_->fisheye_buffer_.addNewData(buffer);
    }
}

void TangoEngine::onPoseAvailable(void* context, const TangoPoseData* pose)
{
    if(pose->status_code == TANGO_POSE_VALID)
    {
        tango_engine_->pose_valid_ = true;
    }
    else
    {
        tango_engine_->pose_valid_ = false;
        return;
    }


    TangoPoseData pose_corrected;

    // This will give a better pose, somehow (averaging of some sort)
    TangoCoordinateFramePair frame_pair;
    frame_pair.base   = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    TangoErrorType tango_error =
            TangoService_getPoseAtTime(pose->timestamp,
                                       frame_pair,
                                       &pose_corrected);



    // Save pose if recording data (only save valid poses)
    tango_engine_->data_writer_.addPoseToSave(&pose_corrected);

}





TangoEngine* TangoEngine::getInstance()
{
    if(!tango_engine_){ tango_engine_.reset(new TangoEngine("")); }
    return tango_engine_.get();
}


TangoEngine::TangoEngine(const char *calibFilename):
        ImageSourceEngine(calibFilename),
        is_initialized_(false),
        pose_valid_(false),
        pc_manager_(nullptr),
        last_depth_timestamp_(-1.0),
        fisheye_buffer_(10, 640, 480)
{
    first_imu_pose_inv_.setIdentity();


    std::string time_filename;

    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    char buffer_time[100];
    strftime(buffer_time, sizeof(buffer_time), "%Y_%m_%d_%H_%M_%S", std::localtime(&now_c));

    data_writer_.setRecordingName(buffer_time);
    data_writer_.setSaveType(DataWriter::SAVE_ALL);
}

TangoEngine::~TangoEngine()
{
    data_writer_.stopRecording();
    if(pc_manager_){ TangoSupport_freePointCloudManager(pc_manager_); }
}


bool TangoEngine::initialize(int32_t max_point_cloud_elements)
{
    // Initialize camera intrinsics
    TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &ccIntrinsics_);
    TangoService_getCameraIntrinsics(TANGO_CAMERA_FISHEYE, &fisheyeIntrinsics_);
    TangoService_getCameraIntrinsics(TANGO_CAMERA_DEPTH, &depthIntr_);

//    LOGI("Color camera intrinsics:");
//    LOGI("Width: %u, Height: %u, fx=%f fy=%f cx=%f cy=%f",
//         ccIntrinsics_.width, ccIntrinsics_.height,
//         ccIntrinsics_.fx, ccIntrinsics_.fy,
//         ccIntrinsics_.cx, ccIntrinsics_.cy);
//
//    LOGI("Fish eye camera intrinsics:");
//    LOGI("Width: %u, Height: %u, fx=%f fy=%f cx=%f cy=%f",
//         fisheyeIntrinsics_.width, fisheyeIntrinsics_.height,
//         fisheyeIntrinsics_.fx, fisheyeIntrinsics_.fy,
//         fisheyeIntrinsics_.cx, fisheyeIntrinsics_.cy);
//
//    LOGI("Depth camera intrinsics:");
//    LOGI("Width: %u, Height: %u, fx=%f fy=%f cx=%f cy=%f",
//         depthIntr_.width, depthIntr_.height,
//         depthIntr_.fx, depthIntr_.fy,
//         depthIntr_.cx, depthIntr_.cy);





    // Initialize point cloud buffer
    if(pc_manager_){ TangoSupport_freePointCloudManager(pc_manager_); }
    TangoSupport_createPointCloudManager(max_point_cloud_elements, &pc_manager_);

    data_writer_.setPointCloudManager(pc_manager_);
    data_writer_.setColorImageBuffer(&color_image_buffer_);
    data_writer_.setFisheyeBuffer(&fisheye_buffer_);
    
    TangoPointCloud* dummy_point_cloud = new TangoPointCloud;
    dummy_point_cloud->num_points = 1;
    dummy_point_cloud->points = new float[max_point_cloud_elements][4];
    dummy_point_cloud->timestamp = -1.0;
    TangoSupport_updatePointCloud(pc_manager_, dummy_point_cloud);

    last_depth_timestamp_  = -1.0;


    // Initialize Image buffer
    color_image_buffer_.initialize(10, ccIntrinsics_.width, ccIntrinsics_.height);


    // Set intrinsics in InfiniTAM format
    calib.intrinsics_rgb.SetFrom(ccIntrinsics_.fx, ccIntrinsics_.fy, ccIntrinsics_.cx, ccIntrinsics_.cy, ccIntrinsics_.width, ccIntrinsics_.height);
    calib.intrinsics_d.SetFrom(depthIntr_.fx, depthIntr_.fy, depthIntr_.cx, depthIntr_.cy, depthIntr_.width, depthIntr_.height);
    calib.disparityCalib.SetFrom(1.0/1000.0, 0.0, ITMLib::Objects::ITMDisparityCalib::TRAFO_AFFINE);

    is_initialized_ = true;

    return true;
}

int TangoEngine::connectCallbacks()
{
    LOGI("connectCallbacks");

    // Attach the onPointCloudAvailable callback.
    int ret = TangoService_connectOnPointCloudAvailable(onPointCloudAvailable);
    if (ret != TANGO_SUCCESS)
    {
        LOGE("Failed to connect to point cloud callback with error code: %d", ret);
        return ret;
    }


    // Pose callback
    TangoCoordinateFramePair pairs;
    pairs.base   = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    pairs.target = TANGO_COORDINATE_FRAME_DEVICE;

    ret = TangoService_connectOnPoseAvailable(1, &pairs, onPoseAvailable);
    if (ret != TANGO_SUCCESS)
    {
        LOGE("Failed to connect to pose callback with error code: %d", ret);
        return ret;
    }


    // Cameras
    ret = TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, this, onColorFrameAvailable);
    if (ret != TANGO_SUCCESS)
    {
        LOGE("Failed to connect to pose callback with error code: %d", ret);
        return ret;
    }

    ret = TangoService_connectOnFrameAvailable(TANGO_CAMERA_FISHEYE, this, onFisheyeFrameAvailable);
    if (ret != TANGO_SUCCESS)
    {
        LOGE("Failed to connect to pose callback with errorcode: %d", ret);
        return ret;
    }

    data_writer_.startRecording();

    return ret;
}


bool TangoEngine::hasMoreImages()
{
    return true;
}

void TangoEngine::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
    TangoPointCloud* point_cloud;
    TangoSupport_getLatestPointCloud(pc_manager_, &point_cloud);

    getRgb(point_cloud->timestamp, rgb);

    convertDepth(point_cloud, rawDepth);

    last_depth_timestamp_ = point_cloud->timestamp;
}

Vector2i TangoEngine::getDepthImageSize()
{
    return Vector2i(depthIntr_.width, depthIntr_.height);
}

Vector2i TangoEngine::getRGBImageSize()
{
    return Vector2i(ccIntrinsics_.width, ccIntrinsics_.height);
}

bool TangoEngine::isImageReady()
{
    TangoPointCloud* point_cloud;
    TangoSupport_getLatestPointCloud(pc_manager_, &point_cloud);

    return (last_depth_timestamp_ != point_cloud->timestamp);
}

bool TangoEngine::getMeasurement(ITMIMUMeasurement *imu, bool first)
{
    return this->getMeasurementAtTimestamp(imu, last_depth_timestamp_, first);
}

bool TangoEngine::getCurrentMeasurement(ITMIMUMeasurement *imu, bool first)
{
    return this->getMeasurementAtTimestamp(imu, 0.0, first);
}

void TangoEngine::setLastCalculatedPose(const ITMPose &pose)
{
    Vector3f poseT = pose.GetT();
    Matrix3f poseR = pose.GetR();
    Vector4f poseQ = ORUtils::QuaternionFromRotationMatrix(poseR);

    TangoPoseData pose_data;
    pose_data.timestamp = last_depth_timestamp_;
    pose_data.orientation[0] = poseQ.x;
    pose_data.orientation[1] = poseQ.y;
    pose_data.orientation[2] = poseQ.z;
    pose_data.orientation[3] = poseQ.w;
    pose_data.translation[0] = poseT.x;
    pose_data.translation[1] = poseT.y;
    pose_data.translation[2] = poseT.z;
    data_writer_.addKinfuPoseToSave(&pose_data);

}

void TangoEngine::stopRecording()
{
    data_writer_.stopRecording();
}



void TangoEngine::getRgb(double timestamp, ITMUChar4Image *rgb)
{
    TangoImageBuffer* buffer = color_image_buffer_.getDataByTimestamp(timestamp);

    Vector4u* data_out = rgb->GetData(MEMORYDEVICE_CPU);

    unsigned width  = ccIntrinsics_.width;
    unsigned height = ccIntrinsics_.height;
    unsigned size   = width * height;

    // NV21 format
    if(buffer->format == TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP)
    {
        const uint8_t* data_in_y  = buffer->data;
        const uint8_t* data_in_uv = data_in_y + size;

        // First scanline is metadata
        for(unsigned j = 0; j < width; j++)
        {
            data_out[j].r = 0;
            data_out[j].g = 0;
            data_out[j].b = 0;
            data_out[j].a = 0;
        }

        for(unsigned i = 1; i < height; i++)
        {
            for(unsigned j = 0; j < width; j++)
            {
                unsigned index_out = (i * width + j);

                unsigned index_in_y = i * width + j;
                unsigned index_in_u = (i/2) * width + (j & ~1) + 1;
                unsigned index_in_v = (i/2) * width + (j & ~1);

                double y = data_in_y[index_in_y];
                double u = data_in_uv[index_in_u];
                double v = data_in_uv[index_in_v];


                uint8_t r = std::min(255.0, std::max(0.0, y + 1.370705 * (v - 128)));
                uint8_t g = std::min(255.0, std::max(0.0, y - 0.337633 * (u - 128) - 0.698001 * (v - 128)));
                uint8_t b = std::min(255.0, std::max(0.0, y + 1.732446 * (u - 128)));


                data_out[index_out].r = r;
                data_out[index_out].g = g;
                data_out[index_out].b = b;
                data_out[index_out].a = 255;
            }
        }
    }

        // YV 12 format
    else if(buffer->format == TANGO_HAL_PIXEL_FORMAT_YV12)
    {
        const uint8_t* data_in_y = buffer->data;
        const uint8_t* data_in_v = data_in_y + size;
        const uint8_t* data_in_u = data_in_v + size/4;

        for(unsigned i = 0; i < height; i++)
        {
            for(unsigned j = 0; j < width; j++)
            {
                unsigned index_out = (i * width + j);

                // WARNING
                // Index calculation not verified!
                //
                unsigned index_in_y = i * width   + j;
                unsigned index_in_u = (i/2) * width/2 + (j/2);
                unsigned index_in_v = (i/2) * width/2 + (j/2);

                double y = data_in_y[index_in_y];
                double u = data_in_u[index_in_u];
                double v = data_in_v[index_in_v];


                uint8_t r = std::min(255.0, std::max(0.0, y + 1.370705 * (v - 128)));
                uint8_t g = std::min(255.0, std::max(0.0, y - 0.337633 * (u - 128) - 0.698001 * (v - 128)));
                uint8_t b = std::min(255.0, std::max(0.0, y + 1.732446 * (u - 128)));


                data_out[index_out].r = r;
                data_out[index_out].g = g;
                data_out[index_out].b = b;
                data_out[index_out].a = 1;
            }
        }

    } // if YV 12
}


void TangoEngine::convertDepth(TangoPointCloud *point_cloud, ITMShortImage *rawDepth)
{

    const float fx = depthIntr_.fx;
    const float fy = depthIntr_.fy;
    const float cx = depthIntr_.cx;
    const float cy = depthIntr_.cy;
    const uint32_t width  = depthIntr_.width;
    const uint32_t height = depthIntr_.height;

    // Reconstructing the depth map from the point cloud:
    // 1. We use the depth camera intrinsics to get the pixel coordinates
    //    in the current view for each point.
    // 2. Because the point cloud is so sparse, we upsample it
    //    either using nearest neighbor, or using a bilateral filter.


    // TODO add this as enum / class member
    const int DEPTH_INTERP_NN        = 1;
    const int DEPTH_INTERP_BILATERAL = 2;
    int depth_interp_type_ = DEPTH_INTERP_NN;

    typedef short depth_t;


    vector<depth_t> depth_map_buffer;

    depth_t* depth_map = rawDepth->GetData(MEMORYDEVICE_CPU);

    // If using NN, directly write into the output buffer.
    if(depth_interp_type_ == DEPTH_INTERP_NN)
    {
        std::fill(depth_map, depth_map + width*height, 0);
    }
    // If using bilateral, first use an intermediate buffer,
    // then in the second part write into the output buffer.
    else
    {
        depth_map_buffer.resize(width * height, 0);
        depth_map = depth_map_buffer.data();
    }


    int box_x1 = -1, box_x2 = 1;
    int box_y1 = -1, box_y2 = 1;

    if(depth_interp_type_ != DEPTH_INTERP_NN)
    {
        box_x1 = 0; box_x2 = 0;
        box_y1 = 0; box_y2 = 0;
    }


    // Project the points to a depth map
    for(uint32_t i = 0; i < point_cloud->num_points; i++)
    {
        const float x = point_cloud->points[i][0];
        const float y = point_cloud->points[i][1];
        const float z = point_cloud->points[i][2];


        depth_t u = (depth_t)roundf((x * fx) / z + cx);
        depth_t v = (depth_t)roundf((y * fy) / z + cy);
        depth_t d = (depth_t)roundf(z * 1000.f); // m to mm


        if(u < width && v < height)
        {
            for(int a = box_y1; a <= box_y2; a++)
            {
                for(int b = box_x1; b <= box_x2; b++)
                {
                    int index = (v+a) * width + u+b;
                    if(index >= 0 && index < width * height)
                    {
                        depth_t value = depth_map[index];
                        //if(value){ d = (value + d) / 2; }
                        if(value){ continue; }
                        depth_map[index] = d;
//                        depth_map[index] = 1000;
                    }
                }
            }
        }
    }


    // Apply a bilateral filter do interpolate depth values
    if(depth_interp_type_ == DEPTH_INTERP_BILATERAL)
    {
        const float sigma_color = 30;     //in mm
        //const float sigma_space = 4.5;     // in pixels
        const float sigma_space = 1;     // in pixels

        const float sigma_space2_inv_half = 0.5f / (sigma_space * sigma_space);
        const float sigma_color2_inv_half = 0.5f / (sigma_color * sigma_color);


//        vector<depth_t> depth_map_interp(width * height);
        depth_t* depth_map_interp = rawDepth->GetData(MEMORYDEVICE_CPU);

        for(int y = 0; y < height; y++)
        {
            for(int x = 0; x < width; x++)
            {
                const int R = static_cast<int>(sigma_space * 1.5);
                const int D = R * 2 + 1;

                int value = depth_map[y * width + x];

                int tx = min (x - D / 2 + D, (int)width - 1);
                int ty = min (y - D / 2 + D, (int)height - 1);

                float sum1 = 0;
                float sum2 = 0;

                for (int cy = max (y - D / 2, 0); cy < ty; ++cy)
                {
                    for (int cx = max (x - D / 2, 0); cx < tx; ++cx)
                    {
                        int tmp = depth_map[cy * width + cx];
                        if(tmp == 0){ continue; }

                        float space2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
                        float color2 = (value - tmp) * (value - tmp);
                        if(value == 0){ color2 = 0; }

                        float weight = exp (-(space2 * sigma_space2_inv_half + color2 * sigma_color2_inv_half));

                        sum1 += tmp * weight;
                        sum2 += weight;
                    }
                }

                int res = roundf(sum1 / sum2);
                depth_map_interp[y * width + x] = max (0, min (res, (int)numeric_limits<depth_t>::max ()));

            } // for width
        } // for height
    } // if using bilateral

}

template<class V4>
V4 quat_mul(V4 q1, V4 q2)
{
    V4 res;
    res.x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
    res.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
    res.z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
    res.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
    return res;
}


bool TangoEngine::getMeasurementAtTimestamp(ITMIMUMeasurement *imu, double timestamp, bool first)
{
    TangoPoseData pose;

    // Get pose in Tango format
    TangoCoordinateFramePair frame_pair;
    frame_pair.base   = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    TangoErrorType tango_error =
            TangoService_getPoseAtTime(timestamp,
                                       frame_pair,
                                       &pose);
    if(tango_error != TANGO_SUCCESS)
    {
        LOGE("Failed to retrieve pose at time %f with error %d", timestamp, tango_error);
        return false;
    }



    // Tango rotation Quaternion
    double qx = pose.orientation[0];
    double qy = pose.orientation[1];
    double qz = pose.orientation[2];
    double qw = pose.orientation[3];

    // Tango position
    double x = pose.translation[0];
    double y = pose.translation[1];
    double z = pose.translation[2];

    if(qx > 9999 || qy > 9999 || qz > 9999 || qw > 9999 || x > 9999 || y > 9999 || z > 9999)
    {
        LOGE("Error: Pose data likely corrupted!");
        LOGE("Quaternion / Pose: %f %f %f %f  %f %f %f", qx, qy, qz, qw, x, y, z);
        return false;
    }


    // Tango pose to 4x4 transformation matrix
    ORUtils::Vector4<double> quat_vector(qx, qy, qz, qw);
    ORUtils::Matrix3<double> tango_rot = ORUtils::QuaternionToRotationMatrix(quat_vector);
    ORUtils::Matrix4<double> tango_pose(  tango_rot.m00, tango_rot.m10, tango_rot.m20, 0,
                                          tango_rot.m01, tango_rot.m11, tango_rot.m21, 0,
                                          tango_rot.m02, tango_rot.m12, tango_rot.m22, 0,
                                          x,             y,             z,             1);


    // Transform pose
    ORUtils::Matrix4<double> rotate90x(1,  0,  0,  0,
                                       0,  0, -1,  0,
                                       0,  1,  0,  0,
                                       0,  0,  0,  1);

    ORUtils::Matrix4<double> rotate180x(1,  0,  0,  0,
                                        0, -1,  0,  0,
                                        0,  0, -1,  0,
                                        0,  0,  0,  1);

    tango_pose = rotate90x * tango_pose * rotate180x;


    // If first pose, save it
    if(first)
    {
        first_imu_pose_inv_ = tango_pose;
    }


    bool good = tango_pose.inv(tango_pose);
    if(!good){ LOGE("Cannot inverse the current pose!"); return false; }



    // Set pose relative to first pose
    tango_pose = tango_pose * first_imu_pose_inv_;


    // Save to IMU object
    imu->R.m00 = tango_pose.m00;
    imu->R.m01 = tango_pose.m01;
    imu->R.m02 = tango_pose.m02;
    imu->R.m10 = tango_pose.m10;
    imu->R.m11 = tango_pose.m11;
    imu->R.m12 = tango_pose.m12;
    imu->R.m20 = tango_pose.m20;
    imu->R.m21 = tango_pose.m21;
    imu->R.m22 = tango_pose.m22;

    imu->t.x = tango_pose.m30;
    imu->t.y = tango_pose.m31;
    imu->t.z = tango_pose.m32;

    return true;
}

















