//
// Created by qkgautier on 1/27/16.
//

#ifndef RECORDSENSORS_DATASAVER_H
#define RECORDSENSORS_DATASAVER_H

#include "tango_client_api.h"
#include "tango_support_api.h"

#include <queue>
#include <thread>
#include <mutex>
#include <atomic>

#include "TangoImages.h"


class DataWriter
{
public:

    static const int SAVE_DEPTH           = 1 << 0;
    static const int SAVE_COLOR_FOR_DEPTH = 1 << 1;
    static const int SAVE_COLOR           = 1 << 2;
    static const int SAVE_FISHEYE         = 1 << 3;
    static const int SAVE_POSE            = 1 << 4;
    static const int SAVE_ALL = SAVE_DEPTH | SAVE_POSE | SAVE_COLOR | SAVE_FISHEYE;

    DataWriter();

    void setRecordingName(const char* name);

    void setSaveType(int type){ save_type_ = type; }

    /**
     * Copy a pose to save it asynchronously.
     * Does nothing if recording is not activated.
     */
    void addPoseToSave(const TangoPoseData* pose);

    /**
     * Copy a Kinfu pose to save it asynchronously.
     * Does nothing if recording is not activated.
     */
    void addKinfuPoseToSave(const TangoPoseData* pose);

    /**
     * Start saving data to file.
     */
    void startRecording();

    /**
     * Stop saving data to file.
     */
    void stopRecording();

    // TODO stop recording when app stops (flush the data)

    void toggleRecording()
    {
        if(is_saving_){ stopRecording(); }
        else{ startRecording(); }
    }

    bool isRecording(){ return is_saving_; }


    /**
     * Set a pointer to the existing point cloud manager.
     */
    void setPointCloudManager(TangoSupportPointCloudManager* manager)
    { pc_manager_ = manager; }


    /**
     * Set a pointer to the existing image buffer.
     */
    void setColorImageBuffer(TangoImages* buffer)
    { color_image_buffer_ = buffer; }


    /**
     * Set a pointer to the existing fisheye buffer.
     */
    void setFisheyeBuffer(TangoImages* buffer)
    { fisheye_buffer_ = buffer; }


public:

    struct DepthTaskOptions
    {
        TangoSupportPointCloudManager* pc_manager;
        std::string pc_dirpath;
        std::vector<double>* pc_timestamps;
        TangoImages* images;
        std::string image_dirpath;
        std::string image_scan_name;

        DepthTaskOptions():
                pc_manager(nullptr), pc_dirpath(""), pc_timestamps(nullptr),
                images(nullptr), image_dirpath(""), image_scan_name("")
        {}
    };


protected:

    int save_type_; ///< What data do we want to write

    std::atomic<bool>is_saving_; ///< Is recording activated

    std::string top_dir_path_; ///< Path to the top folder containing all the recordings
    std::string record_name_; ///< Name of the current recording folder

    std::queue<TangoPoseData> poses_to_save_; ///< Queue of poses to save into a file
    std::mutex pose_mutex_; ///< Protects the queue
    std::string pose_dirpath_;
    std::thread pose_thread_; ///< Thread that saves the poses to file

    std::queue<TangoPoseData> kinfu_poses_to_save_; ///< Queue of kinfu poses to save into a file
    std::mutex kinfu_pose_mutex_; ///< Protects the kinfu queue
    std::thread kinfu_pose_thread_; ///< Thread that saves the kinfu poses to file

    TangoSupportPointCloudManager* pc_manager_; ///< Pointer to the point cloud manager in TangoHandler
    std::string pc_dirpath_; ///< Path to the *directory* where to save the depth data
    std::thread pc_thread_; ///< Thread that saves the point clouds to file
    std::vector<double> pc_timestamps_; ///< Timestamps of saved point clouds

    TangoImages* color_image_buffer_; ///< Pointer to the buffer in TangoHandler

    std::string image_dirpath_; ///< Path to the *directory* where to save the image data
    std::thread image_thread_;

    TangoImages* fisheye_buffer_; ///< Pointer to the buffer in TangoHandler
    std::string fisheye_dirpath_; ///< Path to the *directory* where to save the fisheye data
    std::thread fisheye_thread_;
};


#endif //RECORDSENSORS_DATASAVER_H
