/*
 * OpenCLContext.h
 *
 *  Created on: Aug 25, 2016
 *      Author: qkgautier
 */

#pragma once

#include <vector>
#include <iostream>


#ifdef COMPILE_WITH_OPENCL

#define CL_USE_DEPRECATED_OPENCL_2_0_APIS
#include <CL/cl.h>
#include <CL/cl_ext.h>




#define clCheckErr(x, y) clCheckErr_(x, y, __LINE__, __FILE__)

inline bool clCheckErr_(cl_int err, const char * name, int line = -1, const char* file = NULL)
{
	if (err != CL_SUCCESS)
	{
		std::cerr << "ERROR: " << name << " (" << err << ")" << std::endl;
		if(line >= 0){ std::cerr << "\tAt line " << line << "\n"; }
		if(file){ std::cerr << "\tIn file \"" << file << "\"" << std::endl; }
		return false;
	}

	return true;
}

#endif // COMPILE_WITH_OPENCL



namespace ORUtils
{

class OpenCLContext
{
public:
	enum DeviceType
	{
		DEVICE_CPU,
		DEVICE_GPU,
		DEVICE_ACCELERATOR
	};

	enum
	{
		KERNEL_INTEGRATE_SCENE = 0,
		KERNEL_RAYCAST,
		KERNEL_COMBINED,
		KERNEL_ICP,
		KERNEL_ICP_ACCUMULATE,
		KERNEL_NUM //< Number of kernels. Must be last in enum.
	};

	enum
	{
		AOCX_INTEGRATE_SCENE = 0,
		AOCX_RAYCAST,
		AOCX_COMBINED,
		AOCX_ICP,
		AOCX_INTEGRATE_RAYCAST_ICP,
		AOCX_COMBINED_ICP,
		AOCX_NUM_FILES //< Number of aocx files, must be last in enum
	};

	struct ClContext
	{
#ifdef COMPILE_WITH_OPENCL
		cl_context context;
		cl_device_id device;
		cl_device_type device_type;
		//size_t maxWorkItems[3];

		std::vector<cl_command_queue> queues;

		std::vector<cl_event> events;

		cl_program program;

		// After initialization, kernels[i] == 0 iff kernel i does not exist
		std::vector<cl_kernel> kernels;

#endif
	} cl_context_;


#ifdef COMPILE_WITH_OPENCL
#ifdef OPENCL_DEBUG
	/// This absolutely needs to disappear, terrible, terrible design.
	cl_mem voxelData_cl;
#endif
#endif



public:
	virtual ~OpenCLContext();

	static OpenCLContext& getInstance()
	{
		static OpenCLContext instance;
		return instance;
	}

	bool initialize(DeviceType device = DEVICE_CPU, int fpgaClAlgo = 0);

	bool isInitialized(){ return is_initialized_; }

	bool createAocxProgram(int program_idx);

	bool hasAocxProgram(int program_idx);


	inline bool programAocxKernel(int kernel_idx)
	{
		if(aocx_kernels_[kernel_idx] < 0){ return false; }
		return this->createAocxProgram(aocx_kernels_[kernel_idx]);
	}

	// C++11...
	//OpenCLContext(OpenCLContext const&)  = delete;
	//void operator=(OpenCLContext const&) = delete;

	bool reallocateQueues();


private:
	OpenCLContext();

	// C++03
	OpenCLContext(OpenCLContext const&);
	void operator=(OpenCLContext const&);

private:

	bool is_initialized_;

	std::vector<std::string> aocx_programs_;

	int last_aocx_program_; ///< Last AOCX program that was built

	/// Links a kernel name to an AOCX file
	int aocx_kernels_[KERNEL_NUM];

};

} /* namespace ORUtils */

