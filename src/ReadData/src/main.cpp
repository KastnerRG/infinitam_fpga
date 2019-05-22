

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/meta_registration.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include "colorize.hpp"
#include "depth_map.hpp"
#ifdef WITH_NETWORK
#include "TcpServer.h"
#include "TcpClient.h"
#include "PointCloudNetworkGetter.h"
#endif
#include "TangoData.h"
#include "InfiniTAMServer.h"

#ifdef WITH_GPERFTOOLS
#include <gperftools/profiler.h>
#endif


using namespace tangodata;
using namespace std;
using namespace pcl;
using namespace cimg_library;
using namespace cv;





//********************************************
// Functions
//********************************************

//-------------------------------------------------------
void printUsage(const char* progName)
//-------------------------------------------------------
{
	cout << "Reads input data in the form of (Point cloud / Depth Map [+ RGB] [+ Estimated pose])\n";

	cout << "\n\nUsage: " << progName << " -i <input_dir> [options] \n\n"
			<< "Options:\n"
			<< "-------------------------------------------\n"

			<< "--input-depth <input-dir>    Read input depth maps from the given directory (only if input directory has no point cloud)\n"
			<< "--input-depth-factor <f>     Factor to convert 'input-depth' values to meters (mutually exclusive with 'mm') (default: 0.001)"
			<< "\n"

#ifdef WITH_NETWORK
			<< "--server <port>              Get point clouds from the network (replaces input_dir)\n"
			<< "--server-blocking            Server waits until data are processed before sending response to client\n"
			<< "--client <host:port>         Send point clouds to the network\n"
			<< "\n"
#endif

			<< "--output-img <output-dir>    Output depth map images (in mm) to this directory\n"
			<< "--transparent                Output transparent depth map\n"
			<< "--mm                         Input cloud is in millimeters instead of meters\n"
			<< "\n"

			<< "--scale <factor>             Depth map scaling factor [default: 1]\n"
			<< "--interpolation <num>        Depth interpolation type [0 none, 1 NN, 2 bilinear (default)]\n"
			<< "--kernel <size>              Depth interpolation kernel size in pixels (default: scale)\n"
			<< "--depth-intr w,h,fx,fy,cx,cy Depth intrinsics\n"
			<< "\n"

			<< "--large-cloud                Concatenate all the raw point clouds together using provided poses\n"
			<< "--register-clouds            Register all the clouds and output result"
			<< "\n"

#ifdef USE_INFINITAM
			<< "--infinitam                  Use InfiniTAM\n"
			<< "--infinitam-params <file>    Read InfiniTAM parameters from this file\n"
			<< "\n"
#endif

			<< "--max-frames                 Maximum number of frames to process\n"
			<< "--frames-wait <ms>           Time in ms to wait between frames (approximately) (default: 0)\n"
			<< "\n"

			<< "-h --help                    This help\n"
			<< "\n\n";
}

//-------------------------------------------------------
int main (int argc, char** argv)
//-------------------------------------------------------
{
#ifdef WITH_GPERFTOOLS
	ProfilerStart("profile.log");
#endif

	// Parse input arguments
	//
	if(console::find_argument(argc, argv, "-h") >= 0
			|| console::find_argument(argc, argv, "--help") >= 0)
	{
		printUsage(argv[0]);
		return EXIT_SUCCESS;
	}

	string in_dir_path;
	int in_dir_idx = console::parse(argc, argv, "-i", in_dir_path);

	short server_port = 0;
	bool use_server        = (console::parse(argc, argv, "--server", server_port) >= 0);
	bool server_blocking   = console::find_switch(argc, argv, "--server-blocking");

	if(in_dir_idx < 0 && !use_server)
	{
		printUsage(argv[0]);
		return EXIT_SUCCESS;
	}


	string client_hostport;
	string client_host;
	string client_port;
	bool use_client = (console::parse(argc, argv, "--client", client_hostport) >= 0);
	if(use_client)
	{
		size_t client_hostport_delim = client_hostport.find_last_of(':');
		client_host = client_hostport.substr(0, client_hostport_delim);
		client_port = client_hostport.substr(client_hostport_delim+1, client_hostport.length());
		cout << "Client: " << client_host << " : " << client_port << endl;
	}


	string out_dir_path;
	int out_dir_idx = console::parse(argc, argv, "--output-img", out_dir_path);
	if(out_dir_path.empty()){ out_dir_idx = -1; }
	cout << (out_dir_idx >= 0? "": "Not ") << "Writing depth images" << (out_dir_idx >= 0? string(" to ") + out_dir_path: string()) << endl;


	double upsample_quality = 10;
	bool do_upsample = (console::parse(argc, argv, "--upsample", upsample_quality) >= 0);

	string input_depth_dir = in_dir_path;
	console::parse(argc, argv, "--input-depth", input_depth_dir);

	int interpolation_type = 2;
	int kernel_size = -1;
	float depth_scale = 1.f;
	float points_z_offset = 0.f;
	console::parse(argc, argv, "--kernel", kernel_size);
	console::parse(argc, argv, "--scale", depth_scale);
	console::parse(argc, argv, "--z-offset", points_z_offset);
	if(kernel_size < 1){ kernel_size = roundf(depth_scale); }


	bool depth_transparent = (console::find_argument(argc, argv, "--transparent") >= 0);
	bool set_background    = (console::find_argument(argc, argv, "--background") >= 0);

	console::parse(argc, argv, "--interpolation", interpolation_type);

	DepthInterpolationType interp_type = DEPTH_INTERP_NONE;
	switch(interpolation_type)
	{
	case 1:
		interp_type = DEPTH_INTERP_NN;
		break;
	case 2:
		interp_type = DEPTH_INTERP_BILINEAR;
		break;
	default:
		break;
	}
	cout << "Interpolation type " << interp_type << endl;

	int max_frames = numeric_limits<int>::max();
	console::parse(argc, argv, "--debug", max_frames);
	console::parse(argc, argv, "--max-frames", max_frames);

	unsigned long frames_wait = 0;
	console::parse(argc, argv, "--frames-wait", frames_wait);

	bool use_bigcloud  = (console::find_argument(argc, argv, "--large-cloud") >= 0);
	bool use_infinitam = (console::find_argument(argc, argv, "--infinitam") >= 0);
	bool millimeters   = (console::find_argument(argc, argv, "--mm") >= 0);
	bool register_clouds  = console::find_switch(argc, argv, "--register-clouds");

	float input_depth_factor = 0.001;
	console::parse(argc, argv, "--input-depth-factor", input_depth_factor);

	string infinitam_params_file;
	console::parse(argc, argv, "--infinitam-params", infinitam_params_file);


	string input_depth_intr;
	console::parse(argc, argv, "--depth-intr", input_depth_intr);

	string log_filename;
	ofstream log_file;
	console::parse(argc, argv, "--log", log_filename);

	if(!log_filename.empty())
	{
		log_file.open(log_filename.c_str());
	}


	// Read all data
	//
	vector<SyncedData> alldata;
	bool depth_is_obj = false;
	bool depth_is_img = false;
	if(!use_server)
	{
		cout << "Reading filenames and poses..." << endl;
		TangoData tango_data;
		if(!tango_data.load(in_dir_path, input_depth_dir))
		{
			cerr << "Unable to load data!" << endl;
			return EXIT_FAILURE;
		}
		tango_data.syncFirstTimestamp();
		tango_data.getSyncedData(alldata);
		depth_is_obj = tango_data.isObj();
		depth_is_img = tango_data.isImg();
	}



	Intrinsics depth_intr_;
	depth_intr_.width  = 320;
	depth_intr_.height = 180;
	depth_intr_.fx     = 260.565000;
	depth_intr_.fy     = 260.658000;
	depth_intr_.cx     = 159.792000;
	depth_intr_.cy     = 90.074200;

	if(!input_depth_intr.empty())
	{
		vector<string> strs;
		boost::split(strs, input_depth_intr, boost::is_any_of(","));

		if(strs.size() < 6){ cerr << "Intrinsics error" << endl; return EXIT_FAILURE; }

		depth_intr_.width  = std::stoul(strs[0]);
		depth_intr_.height = std::stoul(strs[1]);
		depth_intr_.fx = std::stod(strs[2]);
		depth_intr_.fy = std::stod(strs[3]);
		depth_intr_.cx = std::stod(strs[4]);
		depth_intr_.cy = std::stod(strs[5]);
	}
	else
	{
		console::print_warn("Warning: No given depth intrinsics, using one Tango device intrinsics!\n");
	}

	cout << "Depth Intrinsics: "
			<< depth_intr_.width << " "
			<< depth_intr_.height << " "
			<< depth_intr_.fx << " "
			<< depth_intr_.fy << " "
			<< depth_intr_.cx << " "
			<< depth_intr_.cy << endl;

	Intrinsics color_intr_ = depth_intr_.scaled(4);
	int color_channels = 3;

	Intrinsics depth_intr_scale = depth_intr_.scaled(depth_scale);

//	const float max_depth = 3000.f;
//	const float min_depth = 300.f;
//	bool reverse_depth = true;
	const float max_depth = 0.f;
	const float min_depth = 0.f;
	bool reverse_depth = false;


	// Large cloud
	PointCloud<PointXYZ>::Ptr big_cloud_ptr(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>& big_cloud = *big_cloud_ptr;


	// InfiniTAM
	InfiniTAMServer::InfiniTAMParams infinitam_params;
	if(!infinitam_params_file.empty()){ infinitam_params = InfiniTAMServer::InfiniTAMParams(infinitam_params_file.c_str()); }
	InfiniTAMServer infinitam(depth_intr_scale.width, depth_intr_scale.height, infinitam_params);


	// Network Server/Client
#ifdef WITH_NETWORK
	PointCloudNetworkGetter pointcloud_network(server_blocking);
	shared_ptr<TcpServer> network_server;
	thread network_server_thread;
	if(use_server)
	{
		network_server_thread = thread(pointCloudServer_task, server_port, &pointcloud_network, ref(network_server));
		if(use_infinitam
				&& infinitam_params.headless
				&& infinitam_params.client_address == "server")
		{
			while(!network_server){ usleep(1000); }
			while(network_server->getSessions().empty()){ usleep(1000); }
			string address = network_server->getSessions().back()->getRemoteAddress();
			cout << "Using client address: " << address << endl;
			infinitam_params.client_address = address;
		}
	}

	thread network_client_thread;
	shared_ptr<TcpClient> network_client;
	vector<float> network_buffer_points;
	if(use_client)
	{
		network_client_thread = thread(
				TcpClient::runClient_task,
				ref(network_client),
				client_host,
				client_port,
				network_server? &network_server->get_io_service(): 0);

		while(!network_client){ usleep(1000); }
		while(!network_client->isConnected()){ usleep(1000); }
	}
#endif

	// Start InfiniTAM
	if(use_infinitam){ infinitam.start(depth_intr_scale.fx, depth_intr_scale.fy, depth_intr_scale.cx, depth_intr_scale.cy); }


	// Point cloud registration
	IterativeClosestPoint<PointXYZ,PointXYZ>::Ptr icp(new IterativeClosestPoint<PointXYZ,PointXYZ>);
	icp->setMaxCorrespondenceDistance(0.05);
	icp->setMaximumIterations(500);
	GeneralizedIterativeClosestPoint<PointXYZ,PointXYZ>::Ptr gicp(new GeneralizedIterativeClosestPoint<PointXYZ,PointXYZ>);
	gicp->setMaximumIterations(500);
	gicp->setMaximumOptimizerIterations(500);
	registration::MetaRegistration<PointXYZ> meta_registration;
	meta_registration.setRegistration(icp);
	vector<Eigen::Matrix4f> registration_transforms;
	PointCloud<PointXYZRGB> pose_points;


	auto identity_pose = Eigen::Affine3f::Identity();

	int frame_n = 0;
	bool main_loop_stop = false;


	// Loop through all depth files
	//
	while(!main_loop_stop)
    {
    	SyncedData synced_data;
    	if(!use_server){ synced_data = alldata[frame_n]; }

        // Read file to point cloud
    	//
        PointCloud<PointXYZ>::Ptr points_ptr(new PointCloud<PointXYZ>);
        PointCloud<PointXYZ>& points = *points_ptr;

#ifdef WITH_NETWORK
        if(use_server)
        {
        	synced_data.timestamp = frame_n;
        	pointcloud_network.waitForData(points_ptr);
        }
        else
#endif
		// Load point cloud data
        if(!depth_is_img)
        {
        	// Disable warnings
        	console::VERBOSITY_LEVEL vlevel = console::getVerbosityLevel();
        	console::setVerbosityLevel(console::L_ERROR);

        	if(depth_is_obj)
        	{
            	pcl::io::loadOBJFile<PointXYZ>(synced_data.depth, points);
        	}
        	else
        	{
            	pcl::io::loadPLYFile<PointXYZ>(synced_data.depth, points);
        	}

        	console::setVerbosityLevel(vlevel);
        }
		// Or load depth map data and convert to point cloud
        else
        {
        	CImg<unsigned int> img(synced_data.depth.c_str());

        	float factor = img._width / depth_intr_.width;

        	Intrinsics intr_scaled = depth_intr_.scaled(factor);

        	depthMapToPointCloud(img.data(), intr_scaled, points, min_depth, max_depth, 1.0/input_depth_factor, reverse_depth);

        	//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
        	//viewer.showCloud (points_ptr);
        	//while (!viewer.wasStopped ()){ }
        }

#ifdef WITH_NETWORK
        if(use_client)
        {
        	network_buffer_points.resize(points.size() * 3, 0.f);
        	for(size_t i = 0; i < points.size(); i++)
        	{
        		network_buffer_points[i*3+0] = points[i].x;
        		network_buffer_points[i*3+1] = points[i].y;
        		network_buffer_points[i*3+2] = points[i].z;
        	}

        	if(frame_n > 0)
        	{
        		while(network_client->getLastResponse() == 0){ usleep(1); } // TODO wait with timeout
        		network_client->resetResponse();
        	}

        	uint64_t buffer_size = network_buffer_points.size() * sizeof(float);
        	network_client->send((const char*)&buffer_size, sizeof(uint64_t));
        	network_client->send((const char*)&network_buffer_points[0], buffer_size);
        }
#endif


        cout << "****************************************************\n";
    	cout << "Frame " << frame_n <<
    			" at timestamp " << synced_data.timestamp <<
				" with " << points.size() << " points\n";
    	cout << synced_data.depth << "\n";
    	cout << synced_data.color << endl;

		if(!log_filename.empty())
		{
			log_file << "Frame " << frame_n << "\n";
			log_file.flush();
		}



        // Anything that has to do with depth map
    	//
        if(out_dir_idx >= 0 || do_upsample || use_infinitam)
        {
        	typedef unsigned short out_depth_t;
        	typedef unsigned short out_mask_t;


			vector<out_mask_t> mask;



			// Find background
			vector<unsigned char> bg_mask;
			Intrinsics bg_intr = depth_intr_.scaled(0.1);
			if(set_background)
			{
				DepthMapCreator dm;
				dm.min_depth = dm.max_depth = 0;
				dm.pointsToDepthMap(points, bg_intr, bg_mask, mask, DEPTH_INTERP_NONE, points_z_offset);

				vector<unsigned char> bg_temp(bg_mask.size());
				dm.processBackgroundMask(bg_mask.data(), bg_intr.width, bg_intr.height, bg_temp.data());
				bg_mask.swap(bg_temp);
			}

			// Output depth as image file
			if(out_dir_idx >= 0)
			{
				// Create the depth map in millimeters
				const float depth_factor = millimeters? 1.f: 1000.f;
				vector<out_depth_t> depth_map(depth_intr_scale.width * depth_intr_scale.height, 0);
				DepthMapCreator dm;
				dm.kernel_size = (uint32_t)kernel_size;
				dm.min_depth = min_depth;
				dm.max_depth = max_depth;
				dm.depth_factor = depth_factor;
				dm.reverse = reverse_depth;
				dm.pointsToDepthMap(points, depth_intr_scale, depth_map, mask, interp_type, points_z_offset);

				if(!bg_mask.empty())
				{
					dm.setBackground(bg_mask.data(), depth_intr_, depth_map.data(), (out_depth_t)1, bg_intr.width, bg_intr.height);
				}

				if(!depth_transparent)
				{
					mask.assign(mask.size(), std::numeric_limits<out_mask_t>::max());
				}


				// Create image and output
				CImg<out_depth_t> img;
				depthMapToTransparentImg(depth_map, mask, depth_intr_scale.width, depth_intr_scale.height, img);

				string out_depth_filename = out_dir_path + string("/depth_") + to_str(synced_data.timestamp) + string(".png");

				img.save(out_depth_filename.c_str());
			}


			if(do_upsample || use_infinitam)
			{
				vector<double> depth_map(depth_intr_scale.width * depth_intr_scale.height, 0);

				// Compute depth map as real values in meters
				DepthMapCreator dm;
				dm.min_depth = dm.max_depth = 0;
				if(millimeters){ dm.depth_factor = 0.001f; }
				else{ dm.depth_factor = 1.f; } // keep values in meters
				if(!do_upsample){ dm.kernel_size = kernel_size; }
				dm.pointsToDepthMap(points, depth_intr_scale, depth_map, mask, do_upsample? DEPTH_INTERP_NONE: interp_type, points_z_offset);

				// Remove NaN and get max depth
			    double max_value = std::numeric_limits<double>::min();
			    for(size_t i = 0; i < depth_map.size(); i++)
			    {
			    	double value = depth_map[i];

			    	if(std::isnan(value)){ depth_map[i] = 0.0; }
			    	else{ max_value = std::max(max_value, value); }
			    }

			    //cout << "Max depth value: " << max_value << endl;

			    cv::Mat dst;
			    dst = cv::Mat(depth_intr_scale.height, depth_intr_scale.width, CV_64F, depth_map.data());


				// InfiniTAM
				if(use_infinitam)
				{
					cv::Mat ref    = cv::imread(synced_data.color, CV_LOAD_IMAGE_COLOR);

					cv::Mat temp_depth;
					dst.convertTo(temp_depth, DataType<unsigned short>::type, 1000.0);

					cv::Mat temp_rgb;
					if(!ref.empty())
					{
						cv::resize(ref, temp_rgb, cv::Size(depth_intr_scale.width, depth_intr_scale.height));
				    	cv::normalize(temp_rgb, temp_rgb, 0, std::numeric_limits<unsigned char>::max(), cv::NORM_MINMAX);
						temp_rgb.convertTo(temp_rgb, DataType<unsigned char>::type);
						cv::cvtColor(temp_rgb, temp_rgb, CV_BGR2RGB);
					}
					else
					{
						//temp_rgb = Mat::zeros(cv::Size(depth_intr_scale.width, depth_intr_scale.height), DataType<unsigned char>::type);
						temp_rgb = Mat::zeros(cv::Size(depth_intr_scale.width, depth_intr_scale.height), CV_8UC3);
					}

			        if(use_infinitam)
			        {
			        	bool last_pose = frame_n >= max_frames-1 || (frame_n >= alldata.size()-1 && !use_server);
			        	infinitam.addNewData((unsigned short*)temp_depth.ptr(), (unsigned char*)temp_rgb.ptr(), &identity_pose, last_pose); // &synced_data.pose
			        }
				}

			}
        }



        // Adding points together
        if(use_bigcloud)
        {
        	PointCloud<PointXYZ>::Ptr transformed_cloud (new PointCloud<PointXYZ> ());
        	pcl::transformPointCloud (points, *transformed_cloud, synced_data.pose);

        	big_cloud += *transformed_cloud;
        }


        // Point cloud registration
        if(register_clouds)
        {
//        	PointCloud<PointXYZ>::Ptr transformed_cloud (new PointCloud<PointXYZ> ());
//        	pcl::transformPointCloud (points, *transformed_cloud, synced_data.pose);

        	if(!meta_registration.registerCloud(points_ptr, (meta_registration.getAbsoluteTransform().inverse() * synced_data.pose).matrix()))
        	{
        		console::print_warn("Warning: Registration failed\n");
        	}
        	Eigen::Matrix4f transform = meta_registration.getAbsoluteTransform();
        	Eigen::Affine3f affine(transform);
        	registration_transforms.push_back(transform);

        	PointXYZRGB pose_point;
        	pose_point.x = affine.translation()[0];
        	pose_point.y = affine.translation()[1];
        	pose_point.z = affine.translation()[2];
        	pose_point.r = pose_point.b = 0;
        	pose_point.g = 255;
        	pose_points.push_back(pose_point);
        }




		frame_n++;

		if(frame_n >= alldata.size() && !use_server){ main_loop_stop = true; }
		if(frame_n >= max_frames)    { main_loop_stop = true; }

		if(frames_wait > 0 && !main_loop_stop){ usleep(frames_wait * 1000); }

    }// for each file


    cout << "Finished processing " << frame_n << " data." << endl;



    if(!big_cloud.empty())
    {
    	PointCloud<PointXYZRGB>::Ptr cloud_out(new PointCloud<PointXYZRGB>);

    	addColors<PointXYZ>(big_cloud_ptr, cloud_out);

    	pcl::io::savePCDFile("large_cloud.pcd", *cloud_out, true);
    }

    if(register_clouds)
    {
    	PointCloud<PointXYZRGB>::Ptr cloud_out(new PointCloud<PointXYZRGB>);

    	PointCloud<PointXYZ>::ConstPtr registered_cloud = meta_registration.getMetaCloud();

    	addColors<PointXYZ>(registered_cloud, cloud_out);

//    	cloud_out->insert(cloud_out->end(), pose_points.begin(), pose_points.end());

    	*cloud_out += pose_points;

    	pcl::io::savePCDFile("registered_cloud.pcd", *cloud_out, true);
    	ofstream pose_file("registered_poses.txt");
    	for(auto& t: registration_transforms)
    	{
    		pose_file << t << "\n";
    	}
    }

	if(use_infinitam){ infinitam.waitUntilExits(); }

#ifdef WITH_NETWORK
	if(network_client)
	{
		network_client->stop();
		network_client_thread.join();
	}
#endif

#ifdef WITH_GPERFTOOLS
	ProfilerStop();
#endif

 	return EXIT_SUCCESS;
}
