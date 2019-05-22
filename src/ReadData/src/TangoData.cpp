/*
 * TangoData.cpp
 *
 *  Created on: Apr 17, 2016
 *      Author: qkgautier
 */

#include "TangoData.h"
#include "utils.h"

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>


using namespace std;
namespace fs = boost::filesystem;


namespace tangodata {

TangoData::TangoData():
		is_obj_(false),
		is_img_(false)
{}

TangoData::~TangoData() {
	// TODO Auto-generated destructor stub
}


void TangoData::clear()
{
	depths_paths_.clear();
	color_paths_.clear();
	poses_.clear();
}


bool TangoData::load(const string& dir_path, std::string input_depth_dir)
{
	this->clear();

	// Read Pose
	{
		fs::path posedir(dir_path);
		fs::path posefilename("pose.txt");
		fs::path posepath = posedir / posefilename;

		double orientation[4];
		double translation[3];
		double timestamp;

		ifstream poseFile(posepath.string());
		if(poseFile.is_open())
		{
			while(true)
			{
				poseFile >> timestamp;

                poseFile >> orientation[0];
                poseFile >> orientation[1];
                poseFile >> orientation[2];
                poseFile >> orientation[3];

                poseFile >> translation[0];
                poseFile >> translation[1];
                poseFile >> translation[2];

                Eigen::Quaternionf   q(orientation[3], orientation[0], orientation[1], orientation[2]);
                Eigen::Translation3f t(
                		Eigen::Vector3f(translation[0], translation[1], translation[2]));

                poses_[timestamp] = Eigen::Affine3f( Eigen::Translation3f(t) * q);

				if(!poseFile.good()){ break; }
			}
		}
		else{ cout << "Warning: No pose file" << endl; }
	}


    // Read depth data
    {

        fs::path depthdir(dir_path);
        fs::directory_iterator end_iter;
        for (fs::directory_iterator itr(depthdir); itr != end_iter; ++itr)
        {
            if (!fs::is_regular_file(itr->path())){ continue; }

            auto current_path     = itr->path();
            auto current_ext      = current_path.extension();
            auto current_filename = current_path.filename();

            if(current_ext.compare(".ply") != 0)
            {
            	if(current_ext.compare(".obj") != 0){ continue; }
            	else{ is_obj_ = true; }
            }


            // Split filename using "_" delimiter
            vector<string> strs;
            boost::split(strs, current_filename.string(), boost::is_any_of("_"));

			double depth_timestamp;
            if(false)
            {
//            	string filename_str = current_filename.string();
//            	filename_str.erase(filename_str.end()-4, filename_str.end());
//            	cout << std::stoul(filename_str) << endl;
            	depth_timestamp = depths_paths_.size();
            }
            else
            {

				if(strs.empty()){ cerr << "Error in depth files." << endl; continue; }

				// Take the last part and remove the ".ply"
				string& time_str = strs.back();
				if(time_str.size() < 4){ cerr << "Error in depth files." << endl; continue; }
				time_str.erase(time_str.end()-4, time_str.end());

				// Convert to a double
				stringstream ss(time_str);
				ss >> depth_timestamp;
            }

            depths_paths_[depth_timestamp] = current_path.string();

        }// for each file
    }


    // Read image filenames
    {
        fs::path imagedir(dir_path);
        fs::directory_iterator end_iter;
        for (fs::directory_iterator itr(imagedir); itr != end_iter; ++itr)
        {
            if (!fs::is_regular_file(itr->path())) { continue; }

            auto current_path     = itr->path();
            auto current_ext      = current_path.extension();
            auto current_filename = current_path.filename();

            if(current_ext.compare(".jpg") != 0){ continue; }

            // Split filename using "_" delimiter
            vector<string> strs;
            boost::split(strs, current_filename.string(), boost::is_any_of("_"));

            if(strs.empty()){ cerr << "Error in image files." << endl; continue; }

			double timestamp;
            if(false)
            {
            	timestamp = color_paths_.size();
            }
            else
            {
				// Check if color image
				if(!has_suffix(strs[strs.size()-2], "image")){ continue; }

				// Take the last part and remove the 3 letters extension
				string& time_str = strs.back();
				if(time_str.size() < 4){ cerr << "Error in image files." << endl; continue; }
				time_str.erase(time_str.end()-4, time_str.end());

				// Convert to a double
				stringstream ss(time_str);
				ss >> timestamp;

            }
			color_paths_[timestamp] = current_path.string();
        }
    }



    // Read other depth data
    if(depths_paths_.empty())
    {
        fs::path depthdir(input_depth_dir);
        fs::directory_iterator end_iter;
        for (fs::directory_iterator itr(depthdir); itr != end_iter; ++itr)
        {
            if (!fs::is_regular_file(itr->path())){ continue; }

            auto current_path     = itr->path();
            auto current_ext      = current_path.extension();
            auto current_filename = current_path.filename();

            if(current_ext.compare(".png") != 0 && current_ext.compare(".jpg") != 0){ continue; }


            // Split filename using "_" delimiter
            vector<string> strs;
            boost::split(strs, current_filename.string(), boost::is_any_of("_"));

            if(strs.empty()){ cerr << "Error in depth files." << endl; continue; }

            // Take the last part and remove the ".ply"
            string& time_str = strs.back();
            if(time_str.size() < 4){ cerr << "Error in depth files." << endl; continue; }
            time_str.erase(time_str.end()-4, time_str.end());

            // Convert to a double
            stringstream ss(time_str);
            double depth_timestamp;
            ss >> depth_timestamp;

            depths_paths_[depth_timestamp] = current_path.string();

        }// for each file
        
    }



	return true;
}


void TangoData::syncFirstTimestamp()
{
	double firstTimestamp = 0;

	// Find first timestamp
	if(!depths_paths_.empty())
	{
		firstTimestamp = std::max(firstTimestamp, depths_paths_.begin()->first);
	}

	if(!color_paths_.empty())
	{
		firstTimestamp = std::max(firstTimestamp, color_paths_.begin()->first);
	}

	if(!poses_.empty())
	{
		firstTimestamp = std::max(firstTimestamp, poses_.begin()->first);
	}


	// Erase everything before first timestamp
	if(!depths_paths_.empty())
	{
		depths_paths_.erase(depths_paths_.begin(), depths_paths_.lower_bound(firstTimestamp));
	}

	if(!color_paths_.empty())
	{
		color_paths_.erase(color_paths_.begin(), color_paths_.lower_bound(firstTimestamp));
	}

	if(!poses_.empty())
	{
		poses_.erase(poses_.begin(), poses_.lower_bound(firstTimestamp));
	}

	if(!input_depth_paths.empty())
	{
		input_depth_paths.erase(input_depth_paths.begin(), input_depth_paths.lower_bound(firstTimestamp));
	}
}


void TangoData::transformPoses()
{
	if(poses_.empty()){ return; }

	Eigen::Affine3f firstPose = poses_.begin()->second;
	Eigen::Affine3f firstPoseInv = firstPose.inverse();

	for(auto it = poses_.begin(); it != poses_.end(); it++)
	{
		it->second = it->second * firstPoseInv;
	}
}



void TangoData::getSyncedData(vector<SyncedData>& alldata, double time_threshold)
{
	alldata.clear();

	alldata.reserve(depths_paths_.size());

	for(auto depth: depths_paths_)
	{
		SyncedData data;
		data.timestamp = depth.first;
		if(input_depth_paths.empty())
		{
			data.depth = depth.second;
		}
		else
		{
			data.depth = input_depth_paths[data.timestamp];
			if(data.depth.empty()){ continue; }
		}

		const Eigen::Affine3f* pose = getDataAtTime(poses_, depth.first, time_threshold);
		if(pose){ data.pose = *pose; }
		else{ data.pose = Eigen::Affine3f::Identity(); }

		const string* color_path = getDataAtTime(color_paths_, depth.first, time_threshold);

		if(!color_paths_.empty())
		{
			if(color_path){ data.color = *color_path; }
			else{ continue; }
		}

		alldata.push_back(data);
	}
}


} /* namespace tangodata */










