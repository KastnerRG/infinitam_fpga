/*
 * InfiniTAMServer.cpp
 *
 *  Created on: Aug 23, 2016
 *      Author: qkgautier
 */

#include "InfiniTAMServer.h"


#ifdef USE_INFINITAM

#include "Engine/UIEngine.h"
#include "Engine/CLIEngine.h"
#ifdef WITH_NETWORK
#include "Engine/NetworkEngine.h"
#endif
#include "Engine/ImageSourceEngine.h"
#include "ITMLib/ITMLib.h"

#include <mutex>
#include <condition_variable>


using namespace InfiniTAM::Engine;
using namespace std;


namespace InfiniTAM
{
	namespace Engine
	{
		class RawImageEngine: public ImageSourceEngine
		{
		public:
			RawImageEngine(float fx, float fy, float cx, float cy, int width, int height):
				ImageSourceEngine(""),
				rgb_(Vector2i(width,height), MEMORYDEVICE_CPU),
				depth_(Vector2i(width,height), MEMORYDEVICE_CPU),
				data_ready_(false), is_last_(false)
			{
				calib.intrinsics_rgb.SetFrom(fx, fy, cx, cy, width, height);
				calib.intrinsics_d.SetFrom(fx, fy, cx, cy, width, height);
				calib.disparityCalib.SetFrom(1.0/1000.0, 0.0, ITMLib::Objects::ITMDisparityCalib::TRAFO_AFFINE);
			}

			virtual bool hasMoreImages(){ return !is_last_; }
			void setLast(){ is_last_ = true; }

			virtual Vector2i getDepthImageSize(){ return depth_.noDims; }
			virtual Vector2i getRGBImageSize(){ return rgb_.noDims; }

			virtual void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
			{
				// Wait until images are ready
				unique_lock<mutex> lock(source_mutex_);
				cv_.wait(lock, [&]{return data_ready_ || is_last_;});

				rgb->SetFrom(&rgb_, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
				rawDepth->SetFrom(&depth_, ORUtils::MemoryBlock<short>::CPU_TO_CPU);

				// Notify images have been read
				data_ready_ = false;
				lock.unlock();
				cv_.notify_all();
			}

			void setImages(const unsigned short* depth, const unsigned char* rgb = 0)
			{
				// Wait until images have been read
				unique_lock<mutex> lock(source_mutex_);
				cv_.wait(lock, [&]{return !data_ready_ || is_last_;});

				// Set RGB
				if(rgb)
				{
					Vector4u* data_out = rgb_.GetData(MEMORYDEVICE_CPU);
					for(size_t i = 0; i < rgb_.dataSize; i++)
					{
						data_out[i].r = rgb[i*3+0];
						data_out[i].g = rgb[i*3+1];
						data_out[i].b = rgb[i*3+2];
						data_out[i].a = 255;
					}
				}

				// Set depth
				short* depth_map = depth_.GetData(MEMORYDEVICE_CPU);
				for(size_t i = 0; i < depth_.dataSize; i++)
				{
					depth_map[i] = depth[i];
				}

				// Notify images are ready
				data_ready_ = true;
				lock.unlock();
				cv_.notify_all();
			}

		private:
			ITMUChar4Image rgb_;
			ITMShortImage depth_;

			mutex source_mutex_;
			condition_variable cv_;
			bool data_ready_;
			bool is_last_;

		}; // RawImageEngine


		class EigenPoseEngine: public IMUSourceEngine
		{
		public:
			EigenPoseEngine():
				IMUSourceEngine(""),
				data_ready_(false), is_last_(false)
			{}

			~EigenPoseEngine(){}
			
			bool hasMoreMeasurements(){ return !is_last_; }
			void setLast(){ is_last_ = true; }


			void getMeasurement(ITMIMUMeasurement *imu)
			{
				// Wait until pose is ready
				unique_lock<mutex> lock(source_mutex_);
				cv_.wait(lock, [&]{return data_ready_ || is_last_;});

				imu->SetFrom(&pose_);

				// Notify pose have been read
				data_ready_ = false;
				lock.unlock();
				cv_.notify_all();
			}

			void setMeasurement(const Eigen::Affine3f* pose)
			{
				// Wait until pose have been read
				unique_lock<mutex> lock(source_mutex_);
				cv_.wait(lock, [&]{return !data_ready_ || is_last_;});

				Eigen::Vector3f trans = pose->translation();
				Eigen::Matrix3f rot = pose->rotation();

				pose_.t = Vector3f(trans.x(), trans.y(), trans.z());
				pose_.R.m00 = rot(0,0);
				pose_.R.m01 = rot(1,0);
				pose_.R.m02 = rot(2,0);
				pose_.R.m10 = rot(0,1);
				pose_.R.m11 = rot(1,1);
				pose_.R.m12 = rot(2,1);
				pose_.R.m20 = rot(0,2);
				pose_.R.m21 = rot(1,2);
				pose_.R.m22 = rot(2,2);

				// Notify pose is ready
				data_ready_ = true;
				lock.unlock();
				cv_.notify_all();
			}

		private:
			ITMIMUMeasurement pose_;

			mutex source_mutex_;
			condition_variable cv_;
			bool data_ready_;
			bool is_last_;

		}; // EigenPoseEngine

	} // namespace
} // namespace


void infinitam_task(
		RawImageEngine* imageSource,
		EigenPoseEngine* imuSource,
		ITMLibSettings* internalSettings,
		ITMMainEngine* mainEngine,
		InfiniTAMServer::InfiniTAMParams* params)
{
	if(params->headless)
	{
#ifdef WITH_NETWORK
		NetworkEngine::Instance()->Initialise(
				imageSource, imuSource, mainEngine,
				internalSettings->deviceType,
				params->client_address, to_string(params->client_port));
		NetworkEngine::Instance()->Run();
		mainEngine->SaveTSDFToFile("world.pcd");
		mainEngine->SavePosesToFile("poses.txt");
		NetworkEngine::Instance()->Shutdown();
#else
		CLIEngine::Instance()->Initialise(imageSource, imuSource, mainEngine, internalSettings->deviceType);
		CLIEngine::Instance()->Run();
		mainEngine->SaveTSDFToFile("world.pcd");
		mainEngine->SavePosesToFile("poses.txt");
		CLIEngine::Instance()->Shutdown();
#endif
	}
	else
	{
		vector<string> arguments = {"InfiniTAM"};

		std::vector<char*> argv;
		for (const auto& arg : arguments)
		    argv.push_back(const_cast<char*>(arg.data()));
		argv.push_back(nullptr);

		int argc = argv.size() - 1;

		UIEngine::Instance()->Initialise(argc, argv.data(), imageSource, imuSource, mainEngine, "./Files/Out", internalSettings->deviceType);
		UIEngine::Instance()->Run();
		mainEngine->SaveTSDFToFile("world.pcd");
		mainEngine->SavePosesToFile("poses.txt");
		UIEngine::Instance()->Shutdown();
	}
}



#endif // USE_INFINITAM


InfiniTAMServer::InfiniTAMServer(int width, int height, InfiniTAMParams params):
	width_(width), height_(height),
	imageSource_     (nullptr),
	imuSource_       (nullptr),
	internalSettings_(nullptr),
	mainEngine_      (nullptr),
	params_(params)
{

}

InfiniTAMServer::~InfiniTAMServer()
{
#ifdef USE_INFINITAM
	if(mainEngine_)      { delete mainEngine_; }
	if(imageSource_)     { delete imageSource_; }
	if(imuSource_)       { delete imuSource_; }
	if(internalSettings_){ delete internalSettings_; }
#endif
}


void InfiniTAMServer::start(float fx, float fy, float cx, float cy)
{
#ifdef USE_INFINITAM

	cout << "Starting InfiniTAM..." << endl;


	ITMLibSettings::DeviceType deviceType = ITMLibSettings::DEVICE_CPU;
	if(params_.device_type == "cpu"){ deviceType = ITMLibSettings::DEVICE_CPU; }
#ifndef COMPILE_WITHOUT_CUDA
	else if(params_.device_type == "gpu"){ deviceType = ITMLibSettings::DEVICE_CUDA; }
#endif
#ifdef COMPILE_WITH_OPENCL
	else if(params_.device_type == "opencl"){ deviceType = ITMLibSettings::DEVICE_OPENCL; }
#endif
	else{ throw std::runtime_error("Wrong device type"); }

	ITMLibSettings::OpenclDeviceType clDeviceType = ITMLibSettings::OPENCL_DEVICE_CPU;
	if(params_.opencl_device == "cpu"){ clDeviceType = ITMLibSettings::OPENCL_DEVICE_CPU; }
	else if(params_.opencl_device == "gpu"){ clDeviceType = ITMLibSettings::OPENCL_DEVICE_GPU; }
	else if(params_.opencl_device == "fpga"){ clDeviceType = ITMLibSettings::OPENCL_DEVICE_ACCELERATOR; }
	else{ throw std::runtime_error("Wrong OpenCL device"); }

//	if(params_.use_opencl)
#ifdef COMPILE_WITH_OPENCL
	{
		ORUtils::OpenCLContext::getInstance().initialize((ORUtils::OpenCLContext::DeviceType)clDeviceType,
				params_.opencl_algo);
	}
#endif

	if(imageSource_){ delete imageSource_; }
	imageSource_ = new RawImageEngine(fx, fy, cx, cy, width_, height_);

	if(imuSource_){ delete imuSource_; }
	imuSource_ = new EigenPoseEngine();

	if(internalSettings_){ delete internalSettings_; }
	if(params_.tracker_type == "color")
	{ internalSettings_ = new ITMLibSettings(ITMLibSettings::TRACKER_COLOR); }
	else if(params_.tracker_type == "icp")
	{ internalSettings_ = new ITMLibSettings(ITMLibSettings::TRACKER_ICP); }
	else if(params_.tracker_type == "ren")
	{ internalSettings_ = new ITMLibSettings(ITMLibSettings::TRACKER_REN); }
	else if(params_.tracker_type == "imu")
	{ internalSettings_ = new ITMLibSettings(ITMLibSettings::TRACKER_IMU); }
	else if(params_.tracker_type == "imu_only")
	{ internalSettings_ = new ITMLibSettings(ITMLibSettings::TRACKER_IMU_ONLY); }
	else if(params_.tracker_type == "wicp")
	{ internalSettings_ = new ITMLibSettings(ITMLibSettings::TRACKER_WICP); }
	else if(params_.tracker_type == "imu_init")
	{ internalSettings_ = new ITMLibSettings(ITMLibSettings::TRACKER_IMU_INIT); }
	else{ throw std::runtime_error("Wrong tracker type"); }

	internalSettings_->sceneParams.voxelSize = params_.voxel_size;
	internalSettings_->sceneParams.mu        = params_.mu;
	internalSettings_->sceneParams.maxW      = params_.max_w;
	internalSettings_->sceneParams.stopIntegratingAtMaxW = params_.use_max_w;
	internalSettings_->sceneParams.viewFrustum_min = 0.4f;
	internalSettings_->sceneParams.viewFrustum_max = 5.0f;

	internalSettings_->useSwapping           = params_.use_swapping;
	internalSettings_->useApproximateRaycast = params_.approximate_raycast;
	internalSettings_->useBilateralFilter    = params_.use_bilateral_filter;
	internalSettings_->deviceType            = deviceType;
	internalSettings_->openclDeviceType      = clDeviceType;

	if(params_.no_hierarchy_levels >= 1)
	{
		internalSettings_->noHierarchyLevels = params_.no_hierarchy_levels;

		if(params_.tracking_regime == "default")
		{ throw std::runtime_error("You must specify tracking regime if you specify the hierarchy levels."); }

		delete[] internalSettings_->trackingRegime;
		internalSettings_->trackingRegime = new TrackerIterationType[internalSettings_->noHierarchyLevels];

		delete[] internalSettings_->noICPIterations;
		internalSettings_->noICPIterations = new int[params_.no_hierarchy_levels];
	}

	if(params_.tracking_regime != "default")
	{
		for(int i = 0; i < internalSettings_->noHierarchyLevels/2; i++)
		{
			if(params_.tracking_regime == "both_only"
					|| params_.tracking_regime == "both_rotation"
							|| params_.tracking_regime == "both_translation")
			{
				internalSettings_->trackingRegime[i] = TRACKER_ITERATION_BOTH;
			}
			else if(params_.tracking_regime == "translation_only")
			{
				internalSettings_->trackingRegime[i] = TRACKER_ITERATION_TRANSLATION;
			}
			else if(params_.tracking_regime == "rotation_only")
			{
				internalSettings_->trackingRegime[i] = TRACKER_ITERATION_ROTATION;
			}
			else{ throw std::runtime_error("Wrong tracking regime"); }
		}

		for(int i = internalSettings_->noHierarchyLevels/2; i < internalSettings_->noHierarchyLevels; i++)
		{
			if(params_.tracking_regime == "both_only")
			{
				internalSettings_->trackingRegime[i] = TRACKER_ITERATION_BOTH;
			}
			else if(params_.tracking_regime == "translation_only" || params_.tracking_regime == "both_translation")
			{
				internalSettings_->trackingRegime[i] = TRACKER_ITERATION_TRANSLATION;
			}
			else if(params_.tracking_regime == "rotation_only" || params_.tracking_regime == "both_rotation")
			{
				internalSettings_->trackingRegime[i] = TRACKER_ITERATION_ROTATION;
			}
			else{ throw std::runtime_error("Wrong tracking regime"); }
		}
	}

	for(int i = 0; i < internalSettings_->noHierarchyLevels; i++)
	{
		internalSettings_->noICPIterations[i] = params_.icp_quality + params_.icp_quality*i;
	}

	internalSettings_->skipPoints = params_.color_skip_points;

	internalSettings_->depthTrackerICPThreshold         = params_.icp_dist_threshold;
	internalSettings_->depthTrackerTerminationThreshold = params_.icp_error_threshold;

	internalSettings_->imuType    = ITMLibSettings::IMU_TANGO;
	internalSettings_->openclAlgo = static_cast<ITMLibSettings::OpenclAlgo>(params_.opencl_algo);


	if(mainEngine_){ delete mainEngine_; }
	mainEngine_ = new ITMMainEngine(internalSettings_, &imageSource_->calib, imageSource_->getRGBImageSize(), imageSource_->getDepthImageSize());

	infinitam_thread_ = thread(infinitam_task, imageSource_, imuSource_, internalSettings_, mainEngine_, &params_);

#endif
}


void InfiniTAMServer::addNewData(const unsigned short* depth, const unsigned char* rgb, Eigen::Affine3f* pose, bool is_last)
{
#ifdef USE_INFINITAM
	int numFrames = mainEngine_->GetNumFrames();

	if(imageSource_)
	{
		imageSource_->setImages(depth, rgb);
		if(is_last){ imageSource_->setLast(); }

	}

	if(imuSource_ && pose)
	{
		if(!params_.pose_from_tango)
		{
			// TODO not working
			Eigen::Quaternion <float> rot180x(0, 1, 0, 0);
			*pose = (*pose) * rot180x;
		}
		imuSource_->setMeasurement(pose);
		if(is_last){ imuSource_->setLast(); }
	}

	if(params_.blocking)
	{
		while(mainEngine_->GetNumFrames() == numFrames){ usleep(1); }
	}
#endif
}


void InfiniTAMServer::waitUntilExits()
{
	if(infinitam_thread_.joinable())
	{
		infinitam_thread_.join();
	}
}



































