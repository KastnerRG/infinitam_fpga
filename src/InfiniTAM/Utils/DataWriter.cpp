//
// Created by qkgautier on 1/27/16.
//

#include "DataWriter.h"
#include "../studio/app/src/main/jni/logging.h"

#include <fstream>
#include <chrono>
#include <iomanip>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>

#include "tinyply.h"


using namespace std;
using namespace tinyply;


template <typename T>
string to_str(const T value, const int precision = 6, bool fixed = false)
{
    std::ostringstream out;
    if(fixed) { out << std::fixed; }
    out << std::setprecision(precision) << value;
    return out.str();
}

void saveImage(TangoImageBuffer* im, const string& dirpath, const string& scan_name)
{
    double t = im->timestamp;

    size_t height = im->height+im->height/2;
    size_t width  = im->width;

    if(scan_name == "fisheye")
    {
        height = im->height;
        width  = im->stride; // stride > width for fisheye
    }

    cv::Mat cvImage(height, width, CV_8UC1, im->data);


    // Fisheye
    if(scan_name == "fisheye"){}
    // NV21 format
    else if(im->format == TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP)
    {
        cv::cvtColor(cvImage, cvImage, CV_YUV2BGR_NV21);
    }
    // YV 12 format
    else if(im->format == TANGO_HAL_PIXEL_FORMAT_YV12)
    {
        cv::cvtColor(cvImage, cvImage, CV_YUV2BGR_YV12);
    }
    else{ }


    cv::Rect roi(0, 0, im->width, im->height);

    cv::imwrite(dirpath + string("/")+ scan_name
                + string("_")+ to_str(t, 2, true)
                + string(".jpg"), cvImage(roi));
}



void save_poses_task(const atomic<bool>& is_saving,
                     queue<TangoPoseData>& pose_queue,
                     mutex& pose_mutex,
                     const string& pose_dirpath,
                     string filename)
{
    string filepath = pose_dirpath + string("/") + filename;

    ofstream fp(filepath);
    if(!fp.is_open())
    {
        LOGE("Could not write to %s", filepath.c_str());
        return;
    }
    fp.precision(8);

    while(is_saving)
    {
        {
            lock_guard <mutex> lock(pose_mutex);

            if (pose_queue.size() > 0)
            {
                // Read the front of the Queue
                TangoPoseData &current_pose = pose_queue.front();

                fp << current_pose.timestamp << " ";
                fp << current_pose.orientation[0] << " " << current_pose.orientation[1] << " ";
                fp << current_pose.orientation[2] << " " << current_pose.orientation[3] << " ";
                fp << current_pose.translation[0] << " " << current_pose.translation[1] << " "
                << current_pose.translation[2] << "\n";

                pose_queue.pop();
            }
        }

        this_thread::sleep_for(chrono::milliseconds(1));
    }
}


void save_depth_task(const atomic<bool>& is_saving,
                     DataWriter::DepthTaskOptions options)
{
    string scan_name = "depth";

    while(is_saving)
    {
        // Get latest point cloud
        TangoPointCloud* point_cloud;
        TangoSupport_getLatestPointCloud(options.pc_manager, &point_cloud);

        // If point cloud hasn't changed, wait for new one
        bool new_data = false;
        while(!new_data)
        {
            this_thread::sleep_for(chrono::milliseconds(1));
            TangoSupport_getLatestPointCloudAndNewDataFlag(options.pc_manager, &point_cloud, &new_data);
        }

        // Save point cloud
        if(!options.pc_dirpath.empty())
        {
            double timestamp = point_cloud->timestamp;

            std::vector<float> points(point_cloud->num_points*3);
            for(unsigned int i = 0; i < point_cloud->num_points; i++)
            {
                points[i*3+0] = point_cloud->points[i][0];
                points[i*3+1] = point_cloud->points[i][1];
                points[i*3+2] = point_cloud->points[i][2];
            }

            std::ofstream file(options.pc_dirpath + string("/")+ scan_name
                                       + string("_")+ to_str(timestamp, 2, true)
                                       + string(".ply"));

            PlyFile plyFile;
            plyFile.add_properties_to_element("vertex", { "x", "y", "z" }, points);
            plyFile.write(file, true);
        }

        // Save image with timestamp closest to point cloud
        if(options.images && !options.image_dirpath.empty() && !options.image_scan_name.empty())
        {
            TangoImageBuffer* im = options.images->getDataByTimestamp(point_cloud->timestamp);
            saveImage(im, options.image_dirpath, options.image_scan_name);
        }

        if(options.pc_timestamps)
        {
            options.pc_timestamps->push_back(point_cloud->timestamp);
        }
    }
}


void save_image_task(const atomic<bool>& is_saving,
                      TangoImages* images,
                      const string& image_dirpath,
                      string scan_name)
{
    double last_timestamp = -1.0;

    while(is_saving)
    {
        TangoImageBuffer* im = images->getCurrentData();

        while(im->timestamp == last_timestamp || im->timestamp == 0.0)
        {
            this_thread::sleep_for(chrono::milliseconds(1));
            im = images->getCurrentData();
        }

        double t = im->timestamp;
        last_timestamp = t;

        saveImage(im, image_dirpath, scan_name);

    }
}





// **************************************************

DataWriter::DataWriter():
        save_type_(SAVE_ALL),
        is_saving_(false),
        top_dir_path_("/sdcard/InfiniTAM/record"),
        pc_manager_(nullptr),
        color_image_buffer_(nullptr),
        fisheye_buffer_(nullptr)
{
    setRecordingName("default");
}

void DataWriter::setRecordingName(const char *name)
{
    record_name_     = name;
    pose_dirpath_    = top_dir_path_ + string("/") + record_name_;
    pc_dirpath_      = top_dir_path_ + string("/") + record_name_;
    image_dirpath_   = top_dir_path_ + string("/") + record_name_;
    fisheye_dirpath_ = top_dir_path_ + string("/") + record_name_;
}


void DataWriter::addPoseToSave(const TangoPoseData *pose)
{
    if(is_saving_)
    {
        lock_guard<mutex> lock(pose_mutex_);
        poses_to_save_.push(*pose);
    }
}


void DataWriter::addKinfuPoseToSave(const TangoPoseData *pose)
{
    if(is_saving_)
    {
        lock_guard<mutex> lock(kinfu_pose_mutex_);
        kinfu_poses_to_save_.push(*pose);
    }
}


void DataWriter::startRecording()
{
    stopRecording();

    is_saving_ = true;

    // Can only create the lower-level folder
    mkdir(pose_dirpath_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    mkdir(pc_dirpath_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    mkdir(image_dirpath_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    mkdir(fisheye_dirpath_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    LOGI("Starting to record");

    // Start pose thread
    if(save_type_ & SAVE_POSE)
    {
        pose_thread_ = thread(save_poses_task,
                              ref(is_saving_),
                              ref(poses_to_save_),
                              ref(pose_mutex_),
                              ref(pose_dirpath_),
                              "pose.txt");
    }

    // Start kinfu pose thread
    if(save_type_ & SAVE_POSE)
    {
        kinfu_pose_thread_ = thread(save_poses_task,
                                    ref(is_saving_),
                                    ref(kinfu_poses_to_save_),
                                    ref(kinfu_pose_mutex_),
                                    ref(pose_dirpath_),
                                    "kinfu_pose.txt");
    }

    // Start depth thread
    if(pc_manager_ &&
            (save_type_ & (SAVE_DEPTH | SAVE_COLOR_FOR_DEPTH)) )
    {
        pc_timestamps_.clear();

        DepthTaskOptions options;
        options.pc_manager    = pc_manager_;
        options.pc_timestamps = &pc_timestamps_;

        if(save_type_ & SAVE_DEPTH)
        {
            options.pc_dirpath = pc_dirpath_;
        }
        if(save_type_ & SAVE_COLOR_FOR_DEPTH)
        {
            options.images          = color_image_buffer_;
            options.image_dirpath   = image_dirpath_;
            options.image_scan_name = "image";
        }

        pc_thread_ = thread(save_depth_task,
                            ref(is_saving_),
                            options);
    }

    // Start image thread
    if(color_image_buffer_ && (save_type_ & SAVE_COLOR))
    {
        image_thread_ = thread(save_image_task,
                               ref(is_saving_),
                               color_image_buffer_,
                               ref(image_dirpath_),
                               "image");
    }

    // Start fisheye thread
    if(fisheye_buffer_ && (save_type_ & SAVE_FISHEYE))
    {
        fisheye_thread_ = thread(save_image_task,
                                 ref(is_saving_),
                                 fisheye_buffer_,
                                 ref(fisheye_dirpath_),
                                 "fisheye");
    }
}


void DataWriter::stopRecording()
{
    if(is_saving_)
    { LOGI("Stop recording"); }

    is_saving_ = false;

    if(pose_thread_.joinable())
    { pose_thread_.join(); }

    if(kinfu_pose_thread_.joinable())
    { kinfu_pose_thread_.join(); }

    if(pc_thread_.joinable())
    {
        pc_thread_.join();

        if(!pc_timestamps_.empty())
        {
            string filepath = pose_dirpath_ + string("/depth_pose.txt");
            ofstream fp(filepath);
            if(!fp.is_open())
            {
                LOGE("Could not write to %s", filepath.c_str());
                return;
            }
            fp.precision(8);

            for(auto& t: pc_timestamps_)
            {
                TangoPoseData current_pose;

                TangoCoordinateFramePair frame_pair;
                frame_pair.base = TANGO_COORDINATE_FRAME_AREA_DESCRIPTION;
                frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
                TangoErrorType tango_error =
                        TangoService_getPoseAtTime(t,
                                                   frame_pair,
                                                   &current_pose);

                fp << current_pose.timestamp << " ";
                fp << current_pose.orientation[0] << " " << current_pose.orientation[1] << " ";
                fp << current_pose.orientation[2] << " " << current_pose.orientation[3] << " ";
                fp << current_pose.translation[0] << " " << current_pose.translation[1] << " "
                << current_pose.translation[2] << "\n";
            }
        }
    }

    if(image_thread_.joinable())
    { image_thread_.join(); }
}



