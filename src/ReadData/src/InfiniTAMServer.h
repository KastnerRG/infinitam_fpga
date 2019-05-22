/*
 * InfiniTAMServer.h
 *
 *  Created on: Aug 23, 2016
 *      Author: qkgautier
 */

#ifndef SRC_INFINITAMSERVER_H_
#define SRC_INFINITAMSERVER_H_

#include <thread>
#include <iostream>

#include <Eigen/Geometry>

#include "SimpleConfig.h"


// Forward declarations
namespace InfiniTAM
{
	namespace Engine
	{
		class RawImageEngine;
		class EigenPoseEngine;
	}
}

namespace ITMLib
{
	namespace Objects
	{
		class ITMLibSettings;
	}
	namespace Engine
	{
		class ITMMainEngine;
	}
}




class InfiniTAMServer
{
public:

	struct InfiniTAMParams
	{
		std::string device_type;
		std::string opencl_device;
		int   opencl_algo;
		bool  use_swapping;
		bool  approximate_raycast;
		bool  use_bilateral_filter;
		std::string tracker_type;
		int   no_hierarchy_levels;
		std::string tracking_regime;
		int   icp_quality;
		bool  color_skip_points;
		float icp_dist_threshold;
		float icp_error_threshold;
		float voxel_size; // meters
		float mu;         // meters
		bool  use_max_w;
		int   max_w;
		bool  pose_from_tango;
		bool  headless;
		std::string client_address;
		int   client_port;
		bool  blocking;

		InfiniTAMParams(
				std::string device = "cpu",
				std::string cl_type = "cpu",
				int algo = 0,
				bool swap = false,
				bool approx = false,
				bool filter = false,
				std::string tracker = "imu_init",
				int levels = -1,
				std::string regime = "default",
				int icp_q = 4,
				bool skip = true,
				float icp_dist = 0.001f,
				float icp_err = 0.001f,
				float vs = 0.01f,
				float mu = 0.02f,
				bool use_maxw = false,
				int  maxw = 100,
				bool pose_tango = true,
				bool hl = false,
				std::string addr = "localhost",
				int port = 4531,
				bool block = false):
					device_type(device),
					opencl_device(cl_type),
					opencl_algo(algo),
					use_swapping(swap),
					approximate_raycast(approx),
					use_bilateral_filter(filter),
					tracker_type(tracker),
					no_hierarchy_levels(levels),
					tracking_regime(regime),
					icp_quality(icp_q),
					color_skip_points(skip),
					icp_dist_threshold(icp_dist),
					icp_error_threshold(icp_err),
					voxel_size(vs),
					mu(mu),
					use_max_w(use_maxw),
					max_w(maxw),
					pose_from_tango(pose_tango),
					headless(hl),
					client_address(addr), client_port(port),
					blocking(block)
		{}

		InfiniTAMParams(const char* filename): InfiniTAMParams()
		{
			SimpleConfig config;
			config.loadConfig(filename);
			config.getValue("device_type", device_type);
			config.getValue("opencl_device", opencl_device);
			config.getValue("opencl_algo", opencl_algo);
			config.getValue("use_swapping", use_swapping);
			config.getValue("approximate_raycast", approximate_raycast);
			config.getValue("use_bilateral_filter", use_bilateral_filter);
			config.getValue("tracker_type", tracker_type);
			config.getValue("no_hierarchy_levels", no_hierarchy_levels);
			config.getValue("tracking_regime", tracking_regime);
			config.getValue("icp_quality", icp_quality);
			config.getValue("color_skip_points", color_skip_points);
			config.getValue("icp_dist_threshold", icp_dist_threshold);
			config.getValue("icp_error_threshold", icp_error_threshold);
			config.getValue("voxel_size", voxel_size);
			config.getValue("mu", mu);
			config.getValue("use_max_w", use_max_w);
			config.getValue("max_w", max_w);
			config.getValue("pose_from_tango", pose_from_tango);
			config.getValue("headless", headless);
			config.getValue("client_address", client_address);
			config.getValue("client_port", client_port);
			config.getValue("blocking", blocking);

			std::vector<std::string> leftovers;
			config.getUnqueriedKeys(leftovers);
			for(const auto& k: leftovers)
			{
				std::cout << "Warning: Unknown key: " << k << std::endl;
			}
		}
	};


public:
	InfiniTAMServer(int width = 1280, int height = 720, InfiniTAMParams params = InfiniTAMParams());

	virtual ~InfiniTAMServer();

	void start(float fx = 1042.26, float fy = 1042.632, float cx = 639.168, float cy = 360.2968);

	void addNewData(const unsigned short* depth, const unsigned char* rgb, Eigen::Affine3f* pose = nullptr, bool is_last = false);

	void waitUntilExits();


private:
	int width_;
	int height_;

	InfiniTAM::Engine::RawImageEngine*  imageSource_;
	InfiniTAM::Engine::EigenPoseEngine* imuSource_;
	ITMLib::Objects::ITMLibSettings*    internalSettings_;
	ITMLib::Engine::ITMMainEngine*      mainEngine_;

	std::thread infinitam_thread_;

	InfiniTAMParams params_;

};

#endif /* SRC_INFINITAMSERVER_H_ */
