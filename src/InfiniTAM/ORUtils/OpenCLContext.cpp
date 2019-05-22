/*
 * OpenCLContext.cpp
 *
 *  Created on: Aug 25, 2016
 *      Author: qkgautier
 */

#include "OpenCLContext.h"

#include <iostream>
#include <fstream>

#include "../Utils/NVTimer.h"
#include "../ITMLib/Utils/ITMLibSettings.h" // TODO ITMLib dependency


using namespace std;

extern "C" const char ITMSceneReconstructionEngine_OpenCL[];
extern "C" const size_t ITMSceneReconstructionEngine_OpenCL_len;


namespace ORUtils
{


#ifdef COMPILE_WITH_OPENCL

#define ReturnError(x) if(!(x)){ return false; }
#define ExitError(x)   if(!(x)){ exit(1); }
#define checkErr(x, y) clCheckErr_(x, y, __LINE__, __FILE__)

#endif // COMPILE_WITH_OPENCL




OpenCLContext::OpenCLContext():is_initialized_(false), last_aocx_program_(-1)
{
	for(int i = 0; i < KERNEL_NUM; i++){ aocx_kernels_[i] = -1; }

#ifdef COMPILE_WITH_OPENCL
#ifdef OPENCL_DEBUG
	/// This absolutely needs to disappear, terrible, terrible design.
	voxelData_cl = 0;
#endif
#endif
}

OpenCLContext::~OpenCLContext()
{}


bool OpenCLContext::initialize(DeviceType device, int openclAlgo)
{
#ifdef COMPILE_WITH_OPENCL


	cl_device_type device_type;
	switch(device)
	{
	case DEVICE_CPU:
		device_type = CL_DEVICE_TYPE_CPU;
		break;
	case DEVICE_GPU:
		device_type = CL_DEVICE_TYPE_GPU;
		break;
	case DEVICE_ACCELERATOR:
		device_type = CL_DEVICE_TYPE_ACCELERATOR;
		break;
	default:
		device_type = CL_DEVICE_TYPE_ALL;
	}


	ClContext* clContext = &cl_context_;


	cout << "Setting up OpenCL..." << endl;


	int err;

	vector<cl_platform_id> platform_ids;
	vector<cl_device_id> device_ids;
	cl_context context;


	// Get platform and devices
	//
	cl_uint num_platforms;
	err = clGetPlatformIDs(0, NULL, &num_platforms);
	ReturnError(checkErr(err, "Failed to get number of platforms!"));

	cout << num_platforms << " platforms:" << endl;

	platform_ids.resize(num_platforms);

	err = clGetPlatformIDs(num_platforms, platform_ids.data(), NULL);
	ReturnError(checkErr(err, "Failed to get platform ID!"));

	for(cl_uint plat = 0; plat < num_platforms; plat++)
	{
		size_t sz;
		err = clGetPlatformInfo(platform_ids[plat], CL_PLATFORM_NAME, 0, NULL, &sz);
		ReturnError(checkErr(err, "Failed to get size of platform name!"));

		char* name = new char[sz];
		err = clGetPlatformInfo(platform_ids[plat], CL_PLATFORM_NAME, sz, name, NULL);
		ReturnError(checkErr(err, "Failed to get platform name!"));

		cout << "  - " << name << endl;


		cl_uint num_devices;
		clGetDeviceIDs(platform_ids[plat], CL_DEVICE_TYPE_ALL, 0, NULL, &num_devices);
		ReturnError(checkErr(err, "Failed to get number of devices!"));

		cout << "    with " << num_devices << " device(s):" << endl;

		device_ids.resize(num_devices);
		err = clGetDeviceIDs(platform_ids[plat], CL_DEVICE_TYPE_ALL, num_devices, device_ids.data(), NULL);
		ReturnError(checkErr(err, "Failed to get devices!"));

		for(cl_uint i = 0; i < num_devices; i++)
		{
			size_t sz;
			clGetDeviceInfo(device_ids[i], CL_DEVICE_NAME, 0, NULL, &sz);
			ReturnError(checkErr(err, "Failed to get size of device name!"));

			char* name = new char[sz];
			clGetDeviceInfo(device_ids[i], CL_DEVICE_NAME, sz, name, NULL);
			ReturnError(checkErr(err, "Failed to get device name!"));

			cout << "      - " << name << endl;

			delete[] name;
		}

	}






	// Connect to a compute device


	for(cl_uint plat = 0; plat < num_platforms; plat++)
	{
		err = clGetDeviceIDs(platform_ids[plat], device_type, 1, device_ids.data(), NULL);
		if(err == CL_SUCCESS){ break; }
	}
	ReturnError(checkErr(err, "Failed to find a device!"));

	{
		size_t sz;
		clGetDeviceInfo(device_ids[0], CL_DEVICE_NAME, 0, NULL, &sz);
		ReturnError(checkErr(err, "Failed to get size of device name!"));

		char* name = new char[sz];
		clGetDeviceInfo(device_ids[0], CL_DEVICE_NAME, sz, name, NULL);
		ReturnError(checkErr(err, "Failed to get device name!"));

		cout << "*** Using " << name << " ***" << endl;

		delete[] name;
	}

	// Create a compute context
	context = clCreateContext(0, 1, device_ids.data(), NULL, NULL, &err);
	ReturnError(checkErr(err, "Failed to create a compute context!"));


	// debug
	{
//		clGetDeviceInfo(device_ids[0], CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(clContext->maxWorkItems), clContext->maxWorkItems, NULL);
//		cout << "Max work items: "
//				<< clContext->maxWorkItems[0] << " "
//				<< clContext->maxWorkItems[1] << " "
//				<< clContext->maxWorkItems[2] << endl;

		cl_ulong sizeMemGlob;
		clGetDeviceInfo(device_ids[0], CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(cl_ulong), &sizeMemGlob, NULL);
		
		cl_ulong sizeMemAlloc;
		clGetDeviceInfo(device_ids[0], CL_DEVICE_MAX_MEM_ALLOC_SIZE, sizeof(cl_ulong), &sizeMemAlloc, NULL);

		cout << "Global memory size: " << sizeMemGlob << " bytes" << endl;
		cout << "Max memory allocation size: " << sizeMemAlloc << " bytes" << endl;
	}


	// Set contexts, build kernels, and create queues/events

	clContext->device      = device_ids[0];
	clContext->context     = context;
	clContext->device_type = device_type;



	clContext->events.resize(KERNEL_NUM);
	clContext->queues.resize(KERNEL_NUM);
	clContext->kernels.resize(KERNEL_NUM, 0);


	// Initialize queues
	for(unsigned int i = 0; i < clContext->queues.size(); i++)
	{
		clContext->queues[i] = clCreateCommandQueue(clContext->context, clContext->device, CL_QUEUE_PROFILING_ENABLE, &err);
		ReturnError(checkErr(err, "Failed to create a command queue!"));
	}





	// Create the compute program from the source buffer / kernel file

	aocx_programs_.clear();

	if(device_type & CL_DEVICE_TYPE_ACCELERATOR)
	{
		const int num_files = AOCX_NUM_FILES;

		const char* AOCX_FILENAMES[num_files] = {
				"ITMSceneReconstructionEngine_OpenCL.aocx",
				"ITMVisualisationEngine_OpenCL.aocx",
				"ITMReconstructionVisualisation_OpenCL.aocx",
				"ITMDepthTracker_OpenCL.aocx",
				"ITMReconstructionVisualisationTracker_OpenCL.aocx",
				"ITMCombinedTracker_OpenCL.aocx"
		};
	
		aocx_programs_.resize(num_files);

		for(int i = 0; i < num_files; i++)
		{
			switch(openclAlgo)
			{
			case ITMLibSettings::OPENCL_ALGO_INTEGRATE_ONLY:
				if(i != AOCX_INTEGRATE_SCENE){ continue; }
				break;
			case ITMLibSettings::OPENCL_ALGO_RAYCAST_ONLY:
				if(i != AOCX_RAYCAST){ continue; }
				break;
			case ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_REPROGRAM:
				if(i != AOCX_INTEGRATE_SCENE && i != AOCX_RAYCAST){ continue; }
				break;
			case ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_COMBINED:
				if(i != AOCX_COMBINED){ continue; }
				break;
			case ITMLibSettings::OPENCL_ALGO_ICP:
				if(i != AOCX_ICP){ continue; }
				break;
			case ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_ICP:
				if(i != AOCX_INTEGRATE_RAYCAST_ICP){ continue; }
				break;
			case ITMLibSettings::OPENCL_ALGO_COMBINED_ICP:
				if(i != AOCX_COMBINED_ICP){ continue; }
				break;
			default:
				break;
			}

			std::ifstream file(AOCX_FILENAMES[i]);

			if(file.is_open())
			{
				aocx_programs_[i] = string(istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));
			}
		}

		for(int i = 0; i < AOCX_NUM_FILES; i++){ createAocxProgram(i); }
	}

	else
	{
		//const char* prog_source = ITMSceneReconstructionEngine_OpenCL;
		//const size_t prog_length = ITMSceneReconstructionEngine_OpenCL_len;

		const char* KERNEL_FILENAME = "ITMSceneReconstructionEngine_OpenCL.cl";
		std::ifstream file(KERNEL_FILENAME);
		string prog;
		if(file.is_open())
		{
			prog = string(istreambuf_iterator<char>(file),
					(std::istreambuf_iterator<char>()));
		}
		const char* prog_source = prog.c_str();

		clContext->program = clCreateProgramWithSource(clContext->context, 1, &prog_source, NULL, &err);
		ReturnError(checkErr(err, "Failed to create compute program!"));


		// Build the program executable

		err = clBuildProgram(clContext->program, 0, NULL, "-I .", NULL, NULL);
		{
			size_t logSize;
			checkErr(clGetProgramBuildInfo(clContext->program, clContext->device, CL_PROGRAM_BUILD_LOG, 0, NULL, &logSize),
					"Get builg log size");

			vector<char> buildLog(logSize);
			checkErr(clGetProgramBuildInfo(clContext->program, clContext->device, CL_PROGRAM_BUILD_LOG, logSize, buildLog.data(), NULL),
					"clGetProgramBuildInfo");

			string log(buildLog.begin(), buildLog.end());
			std::cout
			<< "\n----------------------Kernel build log----------------------\n"
			<< log
			<< "\n------------------------------------------------------------\n"
			<< std::endl;
		}
		ReturnError(checkErr(err, "Failed to build program executable!"));


		// Create kernels


		if(openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_ONLY
				|| openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_REPROGRAM
				|| openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_ICP)
		{
			clContext->kernels[KERNEL_INTEGRATE_SCENE] = clCreateKernel(clContext->program, "IntegrateIntoScene_depth_s", &err);
			if(err == CL_INVALID_KERNEL_NAME){ clContext->kernels[KERNEL_INTEGRATE_SCENE] = 0; err = CL_SUCCESS; }
			ReturnError(checkErr(err, "Failed to create kernel!"));
		}
		if(openclAlgo == ITMLibSettings::OPENCL_ALGO_RAYCAST_ONLY
				|| openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_REPROGRAM
				|| openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_ICP)
		{
			clContext->kernels[KERNEL_RAYCAST] = clCreateKernel(clContext->program, "Raycast_depth_s", &err);
			if(err == CL_INVALID_KERNEL_NAME){ clContext->kernels[KERNEL_RAYCAST] = 0; err = CL_SUCCESS; }
			ReturnError(checkErr(err, "Failed to create kernel!"));
		}

		if(openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_COMBINED
				|| openclAlgo == ITMLibSettings::OPENCL_ALGO_COMBINED_ICP)
		{
			clContext->kernels[KERNEL_COMBINED] = clCreateKernel(clContext->program, "IntegrateRaycastCombined_depth_s", &err);
			if(err == CL_INVALID_KERNEL_NAME){ clContext->kernels[KERNEL_COMBINED] = 0; err = CL_SUCCESS; }
			ReturnError(checkErr(err, "Failed to create kernel!"));
		}

		if(openclAlgo == ITMLibSettings::OPENCL_ALGO_ICP
				|| openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_ICP
				|| openclAlgo == ITMLibSettings::OPENCL_ALGO_COMBINED_ICP)
		{
			clContext->kernels[KERNEL_ICP] = clCreateKernel(clContext->program, "icp", &err);
			if(err == CL_INVALID_KERNEL_NAME){ clContext->kernels[KERNEL_ICP] = 0; err = CL_SUCCESS; }
			ReturnError(checkErr(err, "Failed to create kernel!"));

			clContext->kernels[KERNEL_ICP_ACCUMULATE] = clCreateKernel(clContext->program, "icp_accumulate", &err);
			if(err == CL_INVALID_KERNEL_NAME){ clContext->kernels[KERNEL_ICP_ACCUMULATE] = 0; err = CL_SUCCESS; }
			ReturnError(checkErr(err, "Failed to create kernel!"));
		}
	}






	// Print used kernel
	int num_kernels = 0;
	std::cout << "Kernels using OpenCL: ";
	for(int i = 0; i < KERNEL_NUM; i++)
	{
		if(clContext->kernels[i] != 0){ std::cout << i << "  "; num_kernels++; }
	}
	if(num_kernels > 0){ std::cout << std::endl; }
	else{ std::cout << "WARNING: NO OPENCL KERNEL, USING CPU!\n" << std::endl; }




	is_initialized_ = true;
	std::cout << "OpenCL initialized." << std::endl;

	return true;
#else
	is_initialized_ = false;
	return false;
#endif
}


bool OpenCLContext::createAocxProgram(int program_idx)
{

#ifdef COMPILE_WITH_OPENCL

	if(program_idx == last_aocx_program_){ return true; }

	if(program_idx < 0 || (unsigned int)program_idx >= aocx_programs_.size()){ return false; }

	if(aocx_programs_[program_idx].empty()){ return false; }

	const char* prog_source  = aocx_programs_[program_idx].c_str();
	const size_t prog_length = aocx_programs_[program_idx].size();

	int err;
	cl_context_.program = clCreateProgramWithBinary(cl_context_.context, 1, &cl_context_.device, &prog_length, (const unsigned char**)&prog_source, NULL, &err);
	ReturnError(checkErr(err, "Failed to create compute program!"));

	err = clBuildProgram(cl_context_.program, 0, NULL, "-I .", NULL, NULL);
	ReturnError(checkErr(err, "Failed to build program executable!"));

	vector<string> kernel_names;
	vector<int> kernel_idx;
	switch(program_idx)
	{
	case AOCX_INTEGRATE_SCENE:
		kernel_names.push_back("IntegrateIntoScene_depth_s");
		kernel_idx.push_back(KERNEL_INTEGRATE_SCENE);
		break;
	case AOCX_RAYCAST:
		kernel_names.push_back("Raycast_depth_s");
		kernel_idx.push_back(KERNEL_RAYCAST);
		break;
	case AOCX_COMBINED:
		kernel_names.push_back("IntegrateRaycastCombined_depth_s");
		kernel_idx.push_back(KERNEL_COMBINED);
		break;
	case AOCX_ICP:
		kernel_names.push_back("icp");
		kernel_idx.push_back(KERNEL_ICP);
		kernel_names.push_back("icp_accumulate");
		kernel_idx.push_back(KERNEL_ICP_ACCUMULATE);
		break;
	case AOCX_INTEGRATE_RAYCAST_ICP:
		kernel_names.push_back("IntegrateIntoScene_depth_s");
		kernel_idx.push_back(KERNEL_INTEGRATE_SCENE);
		kernel_names.push_back("Raycast_depth_s");
		kernel_idx.push_back(KERNEL_RAYCAST);
		kernel_names.push_back("icp");
		kernel_idx.push_back(KERNEL_ICP);
		kernel_names.push_back("icp_accumulate");
		kernel_idx.push_back(KERNEL_ICP_ACCUMULATE);
		break;
	case AOCX_COMBINED_ICP:
		kernel_names.push_back("IntegrateRaycastCombined_depth_s");
		kernel_idx.push_back(KERNEL_COMBINED);
		kernel_names.push_back("icp");
		kernel_idx.push_back(KERNEL_ICP);
		kernel_names.push_back("icp_accumulate");
		kernel_idx.push_back(KERNEL_ICP_ACCUMULATE);
		break;
	default:
		return false;
	}

	for(size_t i = 0; i < kernel_names.size(); i++)
	{
		aocx_kernels_[kernel_idx[i]] = program_idx;

		cl_context_.kernels[kernel_idx[i]] = clCreateKernel(cl_context_.program, kernel_names[i].c_str(), &err);
		std::cout << kernel_names[i] << " " << err << std::endl;
		if(err == CL_INVALID_KERNEL_NAME){ cl_context_.kernels[kernel_idx[i]] = 0; err = CL_SUCCESS; }
		ReturnError(checkErr(err, "Failed to create kernel!"));
	}

	last_aocx_program_ = program_idx;

	return true;

#else
	return false;
#endif // #ifdef COMPILE_WITH_OPENCL
}

bool OpenCLContext::hasAocxProgram(int program_idx)
{
	if(program_idx < 0 || (unsigned int)program_idx >= aocx_programs_.size()){ return false; }

	return aocx_programs_[program_idx].size() > 0;
}

bool OpenCLContext::reallocateQueues()
{
#ifdef COMPILE_WITH_OPENCL

	if(!isInitialized()){ return false; }

	int err;

	std::vector<cl_command_queue>& queues =  cl_context_.queues;

	for(unsigned i = 0; i < queues.size(); ++i)
	{
		if(queues[i])
		{
			clReleaseCommandQueue(queues[i]);
		}
	}

	for(unsigned i = 0; i < queues.size(); ++i)
	{
		queues[i] = clCreateCommandQueue(cl_context_.context, cl_context_.device, CL_QUEUE_PROFILING_ENABLE, &err);
		ReturnError(checkErr(err, "Failed to create a command queue!"));
	}

	return true;

#endif // #ifdef COMPILE_WITH_OPENCL
}


} /* namespace ORUtils */
