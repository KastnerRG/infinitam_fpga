// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMDepthTracker.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMDepthTracker_OpenCL : public ITMDepthTracker
		{
		protected:

			int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);



			ORUtils::MemoryBlock<Matrix4f>* approxInvPose_d;
			ORUtils::MemoryBlock<Matrix4f>* scenePose_d;
			ORUtils::MemoryBlock<float>*    sumHessian_d;
			ORUtils::MemoryBlock<float>*    sumNabla_d;
			ORUtils::MemoryBlock<int>*      noValidPoints_d;
			ORUtils::MemoryBlock<float>*    sumF_d;

#ifdef COMPILE_WITH_OPENCL
			cl_mem valid_pipe_w;
			cl_mem sumF_pipe_w;
			cl_mem nabla_pipe0_w;
			cl_mem nabla_pipe1_w;
			cl_mem nabla_pipe2_w;
			cl_mem nabla_pipe3_w;
			cl_mem nabla_pipe4_w;
			cl_mem nabla_pipe5_w;
			cl_mem hessian_pipe0_w;
			cl_mem hessian_pipe1_w;
			cl_mem hessian_pipe2_w;
			cl_mem hessian_pipe3_w;
			cl_mem hessian_pipe4_w;
			cl_mem hessian_pipe5_w;
			cl_mem hessian_pipe6_w;
			cl_mem hessian_pipe7_w;
			cl_mem hessian_pipe8_w;
			cl_mem hessian_pipe9_w;
			cl_mem hessian_pipe10_w;
			cl_mem hessian_pipe11_w;
			cl_mem hessian_pipe12_w;
			cl_mem hessian_pipe13_w;
			cl_mem hessian_pipe14_w;
			cl_mem hessian_pipe15_w;
			cl_mem hessian_pipe16_w;
			cl_mem hessian_pipe17_w;
			cl_mem hessian_pipe18_w;
			cl_mem hessian_pipe19_w;
			cl_mem hessian_pipe20_w;
			cl_mem valid_pipe_r;
			cl_mem sumF_pipe_r;
			cl_mem nabla_pipe0_r;
			cl_mem nabla_pipe1_r;
			cl_mem nabla_pipe2_r;
			cl_mem nabla_pipe3_r;
			cl_mem nabla_pipe4_r;
			cl_mem nabla_pipe5_r;
			cl_mem hessian_pipe0_r;
			cl_mem hessian_pipe1_r;
			cl_mem hessian_pipe2_r;
			cl_mem hessian_pipe3_r;
			cl_mem hessian_pipe4_r;
			cl_mem hessian_pipe5_r;
			cl_mem hessian_pipe6_r;
			cl_mem hessian_pipe7_r;
			cl_mem hessian_pipe8_r;
			cl_mem hessian_pipe9_r;
			cl_mem hessian_pipe10_r;
			cl_mem hessian_pipe11_r;
			cl_mem hessian_pipe12_r;
			cl_mem hessian_pipe13_r;
			cl_mem hessian_pipe14_r;
			cl_mem hessian_pipe15_r;
			cl_mem hessian_pipe16_r;
			cl_mem hessian_pipe17_r;
			cl_mem hessian_pipe18_r;
			cl_mem hessian_pipe19_r;
			cl_mem hessian_pipe20_r;
#endif
			bool pipes_initialized_;



#ifdef OPENCL_DEBUG
			cl_mem depth_cl;
			cl_mem approxInvPose_cl;
			cl_mem scenePose_cl;
			cl_mem pointsMap_cl;
			cl_mem normalsMap_cl;
			cl_mem sumHessian_cl;
			cl_mem sumNabla_cl;
			cl_mem noValidPoints_cl;
			cl_mem sumF_cl;
			int previousLevelId_;
#endif


		public:
			ITMDepthTracker_OpenCL(Vector2i imgSize, TrackerIterationType *trackingRegime,
					int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
					float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine);

			~ITMDepthTracker_OpenCL(void);

#ifdef OPENCL_DEBUG
			double time_kernels;
#endif
		};
	}
}
