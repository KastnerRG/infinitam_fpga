// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker_OpenCL.h"
#include "../../DeviceAgnostic/ITMDepthTracker.h"


#include "../../../../ORUtils/OpenCLContext.h"

#include "ITMSceneReconstructionEngine_OpenCL_knobs.h"


#ifdef OPENCL_DEBUG
#include <sstream>
#include "../../../../Utils/NVTimer.h"
#endif


using namespace ITMLib::Engine;

ITMDepthTracker_OpenCL::ITMDepthTracker_OpenCL(
		Vector2i imgSize, TrackerIterationType *trackingRegime,
		int noHierarchyLevels, int noICPRunTillLevel,
		float distThresh, float terminationThreshold,
		const ITMLowLevelEngine *lowLevelEngine):
				ITMDepthTracker(imgSize, trackingRegime,
						noHierarchyLevels, noICPRunTillLevel,
						distThresh, terminationThreshold,
						lowLevelEngine, MEMORYDEVICE_CPU),
						approxInvPose_d(0),
						scenePose_d(0),
						sumHessian_d(0),
						sumNabla_d(0),
						noValidPoints_d(0),
						sumF_d(0),
						pipes_initialized_(false)
{
#ifdef OPENCL_DEBUG
	time_kernels     = 0;
	depth_cl         = 0;
	approxInvPose_cl = 0;
	scenePose_cl     = 0;
	pointsMap_cl     = 0;
	normalsMap_cl    = 0;
	sumHessian_cl    = 0;
	sumNabla_cl      = 0;
	noValidPoints_cl = 0;
	sumF_cl          = 0;
	previousLevelId_ = -1;
#endif
}

ITMDepthTracker_OpenCL::~ITMDepthTracker_OpenCL(void)
{
	if(approxInvPose_d){ delete approxInvPose_d; }
	if(scenePose_d)    { delete scenePose_d; }
	if(sumHessian_d)   { delete sumHessian_d; }
	if(sumNabla_d)     { delete sumNabla_d; }
	if(noValidPoints_d){ delete noValidPoints_d; }
	if(sumF_d)         { delete sumF_d; }
#if KNOB4_USE_ND_RANGE == 1
	if(pipes_initialized_)
	{
		clReleaseMemObject(valid_pipe_w);
		clReleaseMemObject(sumF_pipe_w);
		clReleaseMemObject(nabla_pipe0_w);
		clReleaseMemObject(nabla_pipe1_w);
		clReleaseMemObject(nabla_pipe2_w);
		clReleaseMemObject(nabla_pipe3_w);
		clReleaseMemObject(nabla_pipe4_w);
		clReleaseMemObject(nabla_pipe5_w);
		clReleaseMemObject(hessian_pipe0_w);
		clReleaseMemObject(hessian_pipe1_w);
		clReleaseMemObject(hessian_pipe2_w);
		clReleaseMemObject(hessian_pipe3_w);
		clReleaseMemObject(hessian_pipe4_w);
		clReleaseMemObject(hessian_pipe5_w);
		clReleaseMemObject(hessian_pipe6_w);
		clReleaseMemObject(hessian_pipe7_w);
		clReleaseMemObject(hessian_pipe8_w);
		clReleaseMemObject(hessian_pipe9_w);
		clReleaseMemObject(hessian_pipe10_w);
		clReleaseMemObject(hessian_pipe11_w);
		clReleaseMemObject(hessian_pipe12_w);
		clReleaseMemObject(hessian_pipe13_w);
		clReleaseMemObject(hessian_pipe14_w);
		clReleaseMemObject(hessian_pipe15_w);
		clReleaseMemObject(hessian_pipe16_w);
		clReleaseMemObject(hessian_pipe17_w);
		clReleaseMemObject(hessian_pipe18_w);
		clReleaseMemObject(hessian_pipe19_w);
		clReleaseMemObject(hessian_pipe20_w);
		clReleaseMemObject(valid_pipe_r);
		clReleaseMemObject(sumF_pipe_r);
		clReleaseMemObject(nabla_pipe0_r);
		clReleaseMemObject(nabla_pipe1_r);
		clReleaseMemObject(nabla_pipe2_r);
		clReleaseMemObject(nabla_pipe3_r);
		clReleaseMemObject(nabla_pipe4_r);
		clReleaseMemObject(nabla_pipe5_r);
		clReleaseMemObject(hessian_pipe0_r);
		clReleaseMemObject(hessian_pipe1_r);
		clReleaseMemObject(hessian_pipe2_r);
		clReleaseMemObject(hessian_pipe3_r);
		clReleaseMemObject(hessian_pipe4_r);
		clReleaseMemObject(hessian_pipe5_r);
		clReleaseMemObject(hessian_pipe6_r);
		clReleaseMemObject(hessian_pipe7_r);
		clReleaseMemObject(hessian_pipe8_r);
		clReleaseMemObject(hessian_pipe9_r);
		clReleaseMemObject(hessian_pipe10_r);
		clReleaseMemObject(hessian_pipe11_r);
		clReleaseMemObject(hessian_pipe12_r);
		clReleaseMemObject(hessian_pipe13_r);
		clReleaseMemObject(hessian_pipe14_r);
		clReleaseMemObject(hessian_pipe15_r);
		clReleaseMemObject(hessian_pipe16_r);
		clReleaseMemObject(hessian_pipe17_r);
		clReleaseMemObject(hessian_pipe18_r);
		clReleaseMemObject(hessian_pipe19_r);
		clReleaseMemObject(hessian_pipe20_r);
	}
#endif

#ifdef OPENCL_DEBUG
	if(depth_cl)        { clReleaseMemObject(depth_cl); }
	if(approxInvPose_cl){ clReleaseMemObject(approxInvPose_cl); }
	if(scenePose_cl)    { clReleaseMemObject(scenePose_cl); }
	if(pointsMap_cl)    { clReleaseMemObject(pointsMap_cl); }
	if(normalsMap_cl)   { clReleaseMemObject(normalsMap_cl); }
	if(sumHessian_cl)   { clReleaseMemObject(sumHessian_cl); }
	if(sumNabla_cl)     { clReleaseMemObject(sumNabla_cl); }
	if(noValidPoints_cl){ clReleaseMemObject(noValidPoints_cl); }
	if(sumF_cl)         { clReleaseMemObject(sumF_cl); }
#endif
}


inline Vector4f interpolateBilinear_withHoles_OpenCL(
		const Vector4f *source,
		const Vector2f & position,
		const Vector2i & imgSize)
{
	Vector4f a, b, c, d;
	Vector4f result;
	Vector2s p;
	Vector2f delta;

	p.x = (short)floor(position.x);
	p.y = (short)floor(position.y);
	delta.x = position.x - (float)p.x;
	delta.y = position.y - (float)p.y;

	a = source[p.x + p.y * imgSize.x];
	b = source[(p.x + 1) + p.y * imgSize.x];
	c = source[p.x + (p.y + 1) * imgSize.x];
	d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	if (a.w < 0 || b.w < 0 || c.w < 0 || d.w < 0)
	{
		result.x = 0; result.y = 0; result.z = 0; result.w = -1.0f;
		return result;
	}

	result.x = ((float)a.x * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.x * delta.x * (1.0f - delta.y) +
		(float)c.x * (1.0f - delta.x) * delta.y + (float)d.x * delta.x * delta.y);
	result.y = ((float)a.y * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.y * delta.x * (1.0f - delta.y) +
		(float)c.y * (1.0f - delta.x) * delta.y + (float)d.y * delta.x * delta.y);
	result.z = ((float)a.z * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.z * delta.x * (1.0f - delta.y) +
		(float)c.z * (1.0f - delta.x) * delta.y + (float)d.z * delta.x * delta.y);
	result.w = ((float)a.w * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.w * delta.x * (1.0f - delta.y) +
		(float)c.w * (1.0f - delta.x) * delta.y + (float)d.w * delta.x * delta.y);

	return result;
}

inline Vector4f nearestNeighbor_OpenCL(
		const Vector4f *source,
		const Vector2f & position,
		const Vector2i & imgSize)
{
	return source[(int)round(position.x) + (int)round(position.y) * imgSize.x];
}





//template<bool shortIteration, bool rotationOnly>
//inline bool computePerPointGH_Depth_OpenCL(
//		float *localNabla, float *localHessian, float & localF,
//		const int & x, const int & y,
//		const float &depth,
//		const Vector2i & viewImageSize, const Vector4f & viewIntrinsics,
//		const Vector2i & sceneImageSize, const Vector4f & sceneIntrinsics,
//		const Matrix4f & approxInvPose, const Matrix4f & scenePose,
//		const Vector4f *pointsMap,
//		const Vector4f *normalsMap, float distThresh)
//{
//	const int noPara = shortIteration ? 3 : 6;
//	//float A[noPara];
//	float A[6];
//	float b;
//
//	if (depth <= 1e-8f) return false; //check if valid -- != 0.0f
//
//	Vector4f tmp3Dpoint, tmp3Dpoint_reproj;
//	Vector3f ptDiff;
//	Vector4f curr3Dpoint, corr3Dnormal;
//	Vector2f tmp2Dpoint;
//
//	tmp3Dpoint.x = depth * ((float(x) - viewIntrinsics.z) / viewIntrinsics.x);
//	tmp3Dpoint.y = depth * ((float(y) - viewIntrinsics.w) / viewIntrinsics.y);
//	tmp3Dpoint.z = depth;
//	tmp3Dpoint.w = 1.0f;
//
//	// transform to previous frame coordinates
//	tmp3Dpoint = approxInvPose * tmp3Dpoint;
//	tmp3Dpoint.w = 1.0f;
//
//	// project into previous rendered image
//	tmp3Dpoint_reproj = scenePose * tmp3Dpoint;
//	if (tmp3Dpoint_reproj.z <= 0.0f) return false;
//	tmp2Dpoint.x = sceneIntrinsics.x * tmp3Dpoint_reproj.x / tmp3Dpoint_reproj.z + sceneIntrinsics.z;
//	tmp2Dpoint.y = sceneIntrinsics.y * tmp3Dpoint_reproj.y / tmp3Dpoint_reproj.z + sceneIntrinsics.w;
//
//	if (!((tmp2Dpoint.x >= 0.0f) && (tmp2Dpoint.x <= sceneImageSize.x - 2) && (tmp2Dpoint.y >= 0.0f) && (tmp2Dpoint.y <= sceneImageSize.y - 2)))
//		return false;
//
//	curr3Dpoint = nearestNeighbor_OpenCL(pointsMap, tmp2Dpoint, sceneImageSize);
//	//curr3Dpoint = interpolateBilinear_withHoles_OpenCL(pointsMap, tmp2Dpoint, sceneImageSize);;
//	if (curr3Dpoint.w < 0.0f) return false;
//
//	ptDiff.x = curr3Dpoint.x - tmp3Dpoint.x;
//	ptDiff.y = curr3Dpoint.y - tmp3Dpoint.y;
//	ptDiff.z = curr3Dpoint.z - tmp3Dpoint.z;
//	float dist = ptDiff.x * ptDiff.x + ptDiff.y * ptDiff.y + ptDiff.z * ptDiff.z;
//
//	if (dist > distThresh) return false;
//
//	corr3Dnormal = nearestNeighbor_OpenCL(normalsMap, tmp2Dpoint, sceneImageSize);
//	//corr3Dnormal = interpolateBilinear_withHoles_OpenCL(normalsMap, tmp2Dpoint, sceneImageSize);
//	//	if (corr3Dnormal.w < 0.0f) return false;
//
//	b = corr3Dnormal.x * ptDiff.x + corr3Dnormal.y * ptDiff.y + corr3Dnormal.z * ptDiff.z;
//
//	// TODO check whether normal matches normal from image, done in the original paper, but does not seem to be required
//	if (shortIteration)
//	{
//		if (rotationOnly)
//		{
//			A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
//			A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
//			A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
//		}
//		else
//		{
//			A[0] = corr3Dnormal.x;
//			A[1] = corr3Dnormal.y;
//			A[2] = corr3Dnormal.z;
//		}
//	}
//	else
//	{
//		A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
//		A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
//		A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
//		A[3] = corr3Dnormal.x;
//		A[4] = corr3Dnormal.y;
//		A[5] = corr3Dnormal.z;
//	}
//
//
//
//	localF = b * b;
//
//	for (int r = 0, counter = 0; r < noPara; r++)
//	{
//		localNabla[r] = b * A[r];
//
//		for (int c = 0; c <= r; c++, counter++)
//		{
//			localHessian[counter] = A[r] * A[c];
//		}
//	}
//
//	return true;
//}


//#include "../../../../Utils/NVTimer.h"

int ITMDepthTracker_OpenCL::ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{

//	StopWatchInterface* debug_timer;
//	sdkCreateTimer(&debug_timer);
//	sdkStartTimer(&debug_timer);

	ORUtils::OpenCLContext& clContextWrapper = ORUtils::OpenCLContext::getInstance();
	clContextWrapper.programAocxKernel(ORUtils::OpenCLContext::KERNEL_ICP);

	ORUtils::OpenCLContext::ClContext& clContext = clContextWrapper.cl_context_;


	Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;
	Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;

	float *depth = viewHierarchyLevel->depth->GetData(MEMORYDEVICE_CPU);
	Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;
	Vector2i viewImageSize = viewHierarchyLevel->depth->noDims;

	if (iterationType == TRACKER_ITERATION_NONE) return 0;

	const bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);
	const bool rotationOnly   = (iterationType == TRACKER_ITERATION_ROTATION);

//#ifdef OPENCL_DEBUG
//	float sumHessian[6 * 6];
//	float sumNabla[6];
//	float sumF = 0.0f;
//	int noValidPoints = 0;
//#endif

	int noPara   = shortIteration ? 3 : 6;
	int noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;








#ifdef OPENCL_DEBUG
//	for (int i = 0; i < noPara; i++) sumNabla[i] = 0.0f;
//	for (int i = 0; i < noParaSQ; i++) sumHessian[i] = 0.0f;
//
//	for (int y = 0; y < viewImageSize.y; y++)
//	{
//		for (int x = 0; x < viewImageSize.x; x++)
//		{
//			float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;
//
//			for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
//			for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;
//
//
//			//			const int noPara = shortIteration ? 3 : 6;
//			//float A[noPara];
//			float A[6];
//			float b;
//			float d = depth[x + y * viewImageSize.x];
//
//			if (d <= 1e-8f) continue; //check if valid -- != 0.0f
//
//			Vector4f tmp3Dpoint, tmp3Dpoint_reproj;
//			Vector3f ptDiff;
//			Vector4f curr3Dpoint, corr3Dnormal;
//			Vector2f tmp2Dpoint;
//
//			tmp3Dpoint.x = d * ((float(x) - viewIntrinsics.z) / viewIntrinsics.x);
//			tmp3Dpoint.y = d * ((float(y) - viewIntrinsics.w) / viewIntrinsics.y);
//			tmp3Dpoint.z = d;
//			tmp3Dpoint.w = 1.0f;
//
//			// transform to previous frame coordinates
//			tmp3Dpoint = approxInvPose * tmp3Dpoint;
//			tmp3Dpoint.w = 1.0f;
//
//			// project into previous rendered image
//			tmp3Dpoint_reproj = scenePose * tmp3Dpoint;
//			if (tmp3Dpoint_reproj.z <= 0.0f) continue;
//			tmp2Dpoint.x = sceneIntrinsics.x * tmp3Dpoint_reproj.x / tmp3Dpoint_reproj.z + sceneIntrinsics.z;
//			tmp2Dpoint.y = sceneIntrinsics.y * tmp3Dpoint_reproj.y / tmp3Dpoint_reproj.z + sceneIntrinsics.w;
//
//			if (!((tmp2Dpoint.x >= 0.0f) && (tmp2Dpoint.x <= sceneImageSize.x - 2) && (tmp2Dpoint.y >= 0.0f) && (tmp2Dpoint.y <= sceneImageSize.y - 2)))
//				continue;
//
//			curr3Dpoint = nearestNeighbor_OpenCL(pointsMap, tmp2Dpoint, sceneImageSize);
//			//curr3Dpoint = interpolateBilinear_withHoles_OpenCL(pointsMap, tmp2Dpoint, sceneImageSize);;
//			if (curr3Dpoint.w < 0.0f) continue;
//
//			ptDiff.x = curr3Dpoint.x - tmp3Dpoint.x;
//			ptDiff.y = curr3Dpoint.y - tmp3Dpoint.y;
//			ptDiff.z = curr3Dpoint.z - tmp3Dpoint.z;
//			float dist = ptDiff.x * ptDiff.x + ptDiff.y * ptDiff.y + ptDiff.z * ptDiff.z;
//
//			if (dist > distThresh[levelId]) continue;
//
//			corr3Dnormal = nearestNeighbor_OpenCL(normalsMap, tmp2Dpoint, sceneImageSize);
//			//corr3Dnormal = interpolateBilinear_withHoles_OpenCL(normalsMap, tmp2Dpoint, sceneImageSize);
//			//	if (corr3Dnormal.w < 0.0f) return false;
//
//			b = corr3Dnormal.x * ptDiff.x + corr3Dnormal.y * ptDiff.y + corr3Dnormal.z * ptDiff.z;
//
//			// TODO check whether normal matches normal from image, done in the original paper, but does not seem to be required
//			if (shortIteration)
//			{
//				if (rotationOnly)
//				{
//					A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
//					A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
//					A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
//				}
//				else
//				{
//					A[0] = corr3Dnormal.x;
//					A[1] = corr3Dnormal.y;
//					A[2] = corr3Dnormal.z;
//				}
//			}
//			else
//			{
//				A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
//				A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
//				A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
//				A[3] = corr3Dnormal.x;
//				A[4] = corr3Dnormal.y;
//				A[5] = corr3Dnormal.z;
//			}
//
//
//
//			localF = b * b;
//
//			for (int r = 0, counter = 0; r < noPara; r++)
//			{
//				localNabla[r] = b * A[r];
//
//				for (int c = 0; c <= r; c++, counter++)
//				{
//					localHessian[counter] = A[r] * A[c];
//				}
//			}
//
//
//
//
//			//			if (isValidPoint)
//			{
//				noValidPoints++; sumF += localF;
//				for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
//				for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
//			}
//		}// x
//	}// y
//
//	float sumF_debug = sumF;
//	int noValidPoints_debug = noValidPoints;
//	float sumHessian_debug[6 * 6];
//	float sumNabla_debug[6];
//	memcpy(sumHessian_debug, sumHessian, 6*6*sizeof(float));
//	memcpy(sumNabla_debug, sumNabla, 6*sizeof(float));




//	std::cout << "*--------------------------------------------*" << std::endl;
//	std::cout << noValidPoints << "   " << sumF << std::endl;
//
//	for (int r = 0, counter = 0; r < noPara; r++)
//	{
//		for (int c = 0; c <= r; c++, counter++)
//		{
//			std::cout << sumHessian[counter] << " ";
//		}
//	}
//	std::cout << std::endl;
//
//	for (int r = 0; r < noPara; r++)
//	{
//		std::cout << sumNabla[r] << " ";
//	}
//	std::cout << std::endl;
//	std::cout << "*--------------------------------------------*" << std::endl;

#endif























	cl_int2   viewImageSize_cl  = *reinterpret_cast<cl_int2*>(&viewImageSize);
	cl_float4 viewIntrinsics_cl = *reinterpret_cast<cl_float4*>(&viewIntrinsics);
	// *** Inverting FX and FY ***
	viewIntrinsics_cl.x = 1.f / viewIntrinsics_cl.x;
	viewIntrinsics_cl.y = 1.f / viewIntrinsics_cl.y;

	cl_int2   sceneImageSize_cl  = *reinterpret_cast<cl_int2*>(&sceneImageSize);
	cl_float4 sceneIntrinsics_cl = *reinterpret_cast<cl_float4*>(&sceneIntrinsics);

	cl_uchar  shortIteration_cl = shortIteration? 1: 0;
	cl_uchar  rotationOnly_cl   = rotationOnly?   1: 0;

	// From column-major to row-major
	Matrix4f approxInvPose_t = approxInvPose.t();
	Matrix4f scenePose_t = scenePose.t();

	if(!approxInvPose_d){ approxInvPose_d = new ORUtils::MemoryBlock<Matrix4f>(1, MEMORYDEVICE_CPU); }
	if(!scenePose_d)    { scenePose_d     = new ORUtils::MemoryBlock<Matrix4f>(1, MEMORYDEVICE_CPU); }
	if(!sumHessian_d)   { sumHessian_d    = new ORUtils::MemoryBlock<float>(6*6, MEMORYDEVICE_CPU); }
	if(!sumNabla_d)     { sumNabla_d      = new ORUtils::MemoryBlock<float>(6, MEMORYDEVICE_CPU); }
	if(!noValidPoints_d){ noValidPoints_d = new ORUtils::MemoryBlock<int>(1, MEMORYDEVICE_CPU); }
	if(!sumF_d)         { sumF_d          = new ORUtils::MemoryBlock<float>(1, MEMORYDEVICE_CPU); }

	*approxInvPose_d->GetData(MEMORYDEVICE_CPU) = approxInvPose_t;
	*scenePose_d->GetData(MEMORYDEVICE_CPU)     = scenePose_t;

	const int qid = ORUtils::OpenCLContext::KERNEL_ICP;
	const int kid = ORUtils::OpenCLContext::KERNEL_ICP;


	int err;

#ifdef OPENCL_DEBUG
	if(!depth_cl)
	{
//		depth_cl = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(float) * viewImageSize.x * viewImageSize.y, NULL, &err);
		depth_cl = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(float) * view->depth->noDims.x * view->depth->noDims.y, NULL, &err);
		if(!clCheckErr(err, "Failed to allocate memory!")){ return 0; }
	}

	if(!approxInvPose_cl)
	{
		approxInvPose_cl = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(float) * 4*4, NULL, &err);
		if(!clCheckErr(err, "Failed to allocate memory!")){ return 0; }
	}

	if(!scenePose_cl)
	{
		scenePose_cl = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(float) * 4*4, NULL, &err);
		if(!clCheckErr(err, "Failed to allocate memory!")){ return 0; }
	}

	if(!pointsMap_cl)
	{
//		pointsMap_cl = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(Vector4f) * sceneImageSize.x * sceneImageSize.y, NULL, &err);
		pointsMap_cl = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(Vector4f) * view->depth->noDims.x * view->depth->noDims.y, NULL, &err);
		if(!clCheckErr(err, "Failed to allocate memory!")){ return 0; }
	}

	if(!normalsMap_cl)
	{
//		normalsMap_cl = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(Vector4f) * sceneImageSize.x * sceneImageSize.y, NULL, &err);
		normalsMap_cl = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(Vector4f) * view->depth->noDims.x * view->depth->noDims.y, NULL, &err);
		if(!clCheckErr(err, "Failed to allocate memory!")){ return 0; }
	}

	if(!sumHessian_cl)
	{
		sumHessian_cl = clCreateBuffer(clContext.context, CL_MEM_READ_WRITE, sizeof(float) * 6*6, NULL, &err);
		if(!clCheckErr(err, "Failed to allocate memory!")){ return 0; }
	}

	if(!sumNabla_cl)
	{
		sumNabla_cl = clCreateBuffer(clContext.context, CL_MEM_READ_WRITE, sizeof(float) * 6, NULL, &err);
		if(!clCheckErr(err, "Failed to allocate memory!")){ return 0; }
	}

	if(!noValidPoints_cl)
	{
		noValidPoints_cl = clCreateBuffer(clContext.context, CL_MEM_READ_WRITE, sizeof(int), NULL, &err);
		if(!clCheckErr(err, "Failed to allocate memory!")){ return 0; }
	}

	if(!sumF_cl)
	{
		sumF_cl = clCreateBuffer(clContext.context, CL_MEM_READ_WRITE, sizeof(float), NULL, &err);
		if(!clCheckErr(err, "Failed to allocate memory!")){ return 0; }
	}


//	clFinish(clContext.queues[qid]);
//
//	std::vector<cl_event> temp_events(5);

	if(previousLevelId_ != levelId)
	{
		err = clEnqueueWriteBuffer(clContext.queues[qid], depth_cl, CL_FALSE, 0, sizeof(float) * viewImageSize.x * viewImageSize.y, (void*)depth, 0, NULL, NULL); //&temp_events[0]);
		if(!clCheckErr(err, "Failed to write data to the device!")){ return 0; }
	}


	if(levelId == viewHierarchy->noLevels - 1 && previousLevelId_ != levelId)
	{
		err = clEnqueueWriteBuffer(clContext.queues[qid], pointsMap_cl, CL_FALSE, 0, sizeof(Vector4f) * sceneImageSize.x * sceneImageSize.y, (void*)pointsMap, 0, NULL, NULL); //&temp_events[1]);
		if(!clCheckErr(err, "Failed to write data to the device!")){ return 0; }

		err = clEnqueueWriteBuffer(clContext.queues[qid], normalsMap_cl, CL_FALSE, 0, sizeof(Vector4f) * sceneImageSize.x * sceneImageSize.y, (void*)normalsMap, 0, NULL, NULL); //&temp_events[2]);
		if(!clCheckErr(err, "Failed to write data to the device!")){ return 0; }
	}

	err = clEnqueueWriteBuffer(clContext.queues[qid], approxInvPose_cl, CL_FALSE, 0, sizeof(float) * 4*4, (void*)approxInvPose_d->GetData(MEMORYDEVICE_CPU), 0, NULL, NULL); //&temp_events[3]);
	if(!clCheckErr(err, "Failed to write data to the device!")){ return 0; }

//	clFinish(clContext.queues[qid]);
//	StopWatchInterface* timer;
//	sdkCreateTimer(&timer);
//	sdkStartTimer(&timer);

	err = clEnqueueWriteBuffer(clContext.queues[qid], scenePose_cl, CL_TRUE, 0, sizeof(float) * 4*4, (void*)scenePose_d->GetData(MEMORYDEVICE_CPU), 0, NULL, NULL); //&temp_events[4]);
	if(!clCheckErr(err, "Failed to write data to the device!")){ return 0; }
	//std::cout << "Write scene pose: " << sizeof(float) * 4*4 << std::endl;

//	sdkStopTimer(&timer);
//	float time = sdkGetTimerValue(&timer);
//
//	std::cout << "Write: " << time << " ms" << std::endl;
//
//	for(size_t i = 0; i < temp_events.size(); i++)
//	{
//		cl_ulong t_time_start, t_time_end;
//		clGetEventProfilingInfo(temp_events[i], CL_PROFILING_COMMAND_START, sizeof(t_time_start), &t_time_start, NULL);
//		clGetEventProfilingInfo(temp_events[i], CL_PROFILING_COMMAND_END, sizeof(t_time_end), &t_time_end, NULL);
//		double t_time = (t_time_end - t_time_start) / 1000000.0;
//		std::cout << "Write time " << i << ": " << t_time << " ms\n";
//	}
//
//	sdkDeleteTimer(&timer);


#else
	cl_mem depth_cl = viewHierarchyLevel->depth->GetOpenCLData();
	cl_mem approxInvPose_cl = approxInvPose_d->GetOpenCLData();
	cl_mem scenePose_cl = scenePose_d->GetOpenCLData();
	cl_mem pointsMap_cl = sceneHierarchyLevel->pointsMap->GetOpenCLData();
	cl_mem normalsMap_cl = sceneHierarchyLevel->normalsMap->GetOpenCLData();
	cl_mem sumHessian_cl = sumHessian_d->GetOpenCLData();
	cl_mem sumNabla_cl = sumNabla_d->GetOpenCLData();
	cl_mem noValidPoints_cl = noValidPoints_d->GetOpenCLData();
	cl_mem sumF_cl = sumF_d->GetOpenCLData();
#endif





	int argid = 0;

#if KNOB4_USE_ND_RANGE == 1
	if(!pipes_initialized_)
	{
		valid_pipe_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_uchar), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		sumF_pipe_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		nabla_pipe0_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		nabla_pipe1_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		nabla_pipe2_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		nabla_pipe3_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		nabla_pipe4_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		nabla_pipe5_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe0_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe1_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe2_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe3_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe4_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe5_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe6_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe7_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe8_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe9_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe10_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe11_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe12_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe13_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe14_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe15_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe16_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe17_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe18_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe19_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe20_w = clCreatePipe(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }

		valid_pipe_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_uchar), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		sumF_pipe_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		nabla_pipe0_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		nabla_pipe1_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		nabla_pipe2_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		nabla_pipe3_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		nabla_pipe4_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		nabla_pipe5_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe0_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe1_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe2_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe3_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe4_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe5_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe6_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe7_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe8_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe9_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe10_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe11_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe12_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe13_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe14_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe15_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe16_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe17_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe18_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe19_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		hessian_pipe20_r = clCreatePipe(clContext.context, CL_MEM_READ_ONLY, sizeof(cl_float), KNOB4_PIPE_DEPTH, NULL, &err);
		if(!clCheckErr(err, "Failed to create pipe!")){ return 0; }
		pipes_initialized_ = true;
	}

	const int kid2 = ORUtils::OpenCLContext::KERNEL_ICP_ACCUMULATE;
	const int qid2 = ORUtils::OpenCLContext::KERNEL_ICP_ACCUMULATE;


	int totalPoints = viewImageSize.x * viewImageSize.y;

	argid = 0;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&sumHessian_cl); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&sumNabla_cl); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&noValidPoints_cl); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&sumF_cl); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_int),  (void*)&totalPoints); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_uchar),(void*)&shortIteration_cl); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&valid_pipe_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&sumF_pipe_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&nabla_pipe0_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&nabla_pipe1_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&nabla_pipe2_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&nabla_pipe3_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&nabla_pipe4_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&nabla_pipe5_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe0_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe1_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe2_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe3_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe4_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe5_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe6_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe7_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe8_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe9_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe10_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe11_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe12_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe13_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe14_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe15_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe16_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe17_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe18_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe19_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe20_w); argid++;

	const cl_uint work_dim2 = 1;
	size_t localWorkSize2[work_dim2] = {1};
	size_t workSize2[work_dim2] = {1};

	err = clEnqueueNDRangeKernel(
			clContext.queues[qid2], clContext.kernels[kid2], work_dim2, NULL,
			workSize2, localWorkSize2, 0, NULL, &clContext.events[kid2]);
	if(!clCheckErr(err, "Failed to execute kernel!")){ return 0; }
#endif



	argid = 0;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem),  (void*)&depth_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_int2), (void*)&viewImageSize_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_float4), (void*)&viewIntrinsics_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem),  (void*)&approxInvPose_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem),  (void*)&scenePose_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_int2), (void*)&sceneImageSize_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_float4), (void*)&sceneIntrinsics_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem),  (void*)&pointsMap_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem),  (void*)&normalsMap_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_float),(void*)&distThresh[levelId]); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_uchar),(void*)&shortIteration_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_uchar),(void*)&rotationOnly_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem),  (void*)&sumHessian_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem),  (void*)&sumNabla_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem),  (void*)&noValidPoints_cl); argid++;
	clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem),  (void*)&sumF_cl); argid++;

#if KNOB4_USE_ND_RANGE == 1
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&valid_pipe_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&sumF_pipe_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&nabla_pipe0_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&nabla_pipe1_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&nabla_pipe2_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&nabla_pipe3_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&nabla_pipe4_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&nabla_pipe5_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe0_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe1_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe2_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe3_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe4_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe5_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe6_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe7_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe8_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe9_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe10_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe11_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe12_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe13_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe14_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe15_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe16_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe17_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe18_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe19_w); argid++;
	clSetKernelArg(clContext.kernels[kid2], argid, sizeof(cl_mem),  (void*)&hessian_pipe20_w); argid++;

	const cl_uint work_dim = 3;
	size_t localWorkSize[work_dim] = {WORK_GROUP_SIZE_X4, WORK_GROUP_SIZE_Y4, WORK_GROUP_SIZE_Z4};
	size_t workSize[work_dim] = {localWorkSize[0], localWorkSize[1], localWorkSize[2]};
#else

	const cl_uint work_dim = 1;
	size_t localWorkSize[work_dim] = {1};
	size_t workSize[work_dim] = {1};

#endif


//	std::cout << "Executing ICP kernel..." << std::endl;
	err = clEnqueueNDRangeKernel(
			clContext.queues[qid], clContext.kernels[kid], work_dim, NULL,
			workSize, localWorkSize, 0, NULL, &clContext.events[0]);

#if KNOB4_USE_ND_RANGE == 1
	clFinish(clContext.queues[qid2]);
#else
#ifndef OPENCL_DEBUG
	clFinish(clContext.queues[qid]);
#endif
#endif

	if(!clCheckErr(err, "Failed to execute kernel!")){ return 0; }

//	std::cout << "ICP kernel done." << std::endl;


#ifdef OPENCL_DEBUG

	err = clEnqueueReadBuffer(
			clContext.queues[qid], sumHessian_cl, CL_FALSE, 0,
			sizeof(float) * 6*6, (void*)sumHessian_d->GetData(MEMORYDEVICE_CPU), 0, NULL, NULL);
	if(!clCheckErr(err, "Failed to read data from the device!")){ return 0; }

	err = clEnqueueReadBuffer(
			clContext.queues[qid], sumNabla_cl, CL_FALSE, 0,
			sizeof(float) * 6, (void*)sumNabla_d->GetData(MEMORYDEVICE_CPU), 0, NULL, NULL);
	if(!clCheckErr(err, "Failed to read data from the device!")){ return 0; }

	err = clEnqueueReadBuffer(
			clContext.queues[qid], noValidPoints_cl, CL_FALSE, 0,
			sizeof(int), (void*)noValidPoints_d->GetData(MEMORYDEVICE_CPU), 0, NULL, NULL);
	if(!clCheckErr(err, "Failed to read data from the device!")){ return 0; }

	err = clEnqueueReadBuffer(
			clContext.queues[qid], sumF_cl, CL_TRUE, 0,
			sizeof(float), (void*)sumF_d->GetData(MEMORYDEVICE_CPU), 0, NULL, NULL);
	if(!clCheckErr(err, "Failed to read data from the device!")){ return 0; }


	// ----
	// Kernel profiling with OpenCL
	// TODO profile both
	//-----
	cl_ulong k_time_start, k_time_end;
	clGetEventProfilingInfo(clContext.events[0], CL_PROFILING_COMMAND_START, sizeof(k_time_start), &k_time_start, NULL);
	clGetEventProfilingInfo(clContext.events[0], CL_PROFILING_COMMAND_END, sizeof(k_time_end), &k_time_end, NULL);
	double kernel_time = (k_time_end - k_time_start) / 1000000.0;
	time_kernels += kernel_time;
	//std::cout << "Tracking1_Kernel: " << kernel_time << " ms\n";


	previousLevelId_ = levelId;

#endif


	float* sumHessian  = sumHessian_d->GetData(MEMORYDEVICE_CPU);
	float* sumNabla    = sumNabla_d->GetData(MEMORYDEVICE_CPU);
	float& sumF        = *sumF_d->GetData(MEMORYDEVICE_CPU);
	int& noValidPoints = *noValidPoints_d->GetData(MEMORYDEVICE_CPU);



//	std::cout << "--------------------------------------------" << std::endl;
//	std::cout << noValidPoints << "   " << sumF << std::endl;
//
//
//	for (int r = 0, counter = 0; r < noPara; r++)
//	{
//		for (int c = 0; c <= r; c++, counter++)
//		{
//			std::cout << sumHessian[counter] << " ";
//		}
//	}
//	std::cout << std::endl;
//
//
//	for (int r = 0; r < noPara; r++)
//	{
//		std::cout << sumNabla[r] << " ";
//	}
//	std::cout << std::endl;
//	std::cout << "--------------------------------------------\n" << std::endl;





	for (int r = 0, counter = 0; r < noPara; r++)
	{
		for (int c = 0; c <= r; c++, counter++)
		{
			hessian[r + c * 6] = sumHessian[counter];
		}
	}

	for (int r = 0; r < noPara; ++r)
	{
		for (int c = r + 1; c < noPara; c++)
		{
			hessian[r + c * 6] = hessian[c + r * 6];
		}
	}
	
	memcpy(nabla, sumNabla, noPara * sizeof(float));
	f = (noValidPoints > 100) ? sqrt(sumF) / noValidPoints : 1e5f;




#ifdef OPENCL_DEBUG


////	float sumHessian_debug[6 * 6];
////	float sumNabla_debug[6];
//
//	float rmse_hessian = 0.f;
//	float rmse_nabla   = 0.f;
//
//	int counter = 0;
//	for (int r = 0; r < noPara; r++)
//	{
//		for (int c = 0; c <= r; c++, counter++)
//		{
//			float diff = (sumHessian[counter] - sumHessian_debug[counter]);
//			rmse_hessian += diff*diff;
////			std::cout << "localHessian[" << counter << "] = A[" << r << "] * A[" << c << "];" << std::endl;
//		}
//	}
//	rmse_hessian /= counter;
//	rmse_hessian = sqrt(rmse_hessian);
//
//	for (int r = 0; r < noPara; ++r)
//	{
//		float diff = (sumNabla[r] - sumNabla_debug[r]);
//		rmse_nabla += diff*diff;
//	}
//	rmse_nabla /= noPara;
//	rmse_nabla = sqrt(rmse_nabla);
//
//	int error_noPoints = abs(noValidPoints - noValidPoints_debug);
//	float error_sumF   = fabs(sumF - sumF_debug);
//
//	std::stringstream ss;
//	ss << std::fixed;
//	ss.precision(3);
//	ss << "RMSE: Hessian = " << rmse_hessian << " Nabla = " << rmse_nabla
//			<< " Num Points = " << error_noPoints << " SumF = " << error_sumF
//			<< " Iter type = " << shortIteration << " " << rotationOnly;
//	std::cout << ss.str() << std::endl;




//	if(fabs(rmse_hessian + rmse_nabla + error_noPoints + error_sumF) > 0.001f)
//	{
//		std::cout << "*--------------GROUND TRUTH------------------------------*" << std::endl;
//		std::cout << noValidPoints_debug << "   " << sumF_debug << std::endl;
//
//		for (int r = 0, counter = 0; r < noPara; r++)
//		{
//			for (int c = 0; c <= r; c++, counter++)
//			{
//				std::cout << sumHessian_debug[counter] << " ";
//			}
//		}
//		std::cout << std::endl;
//
//		for (int r = 0; r < noPara; r++)
//		{
//			std::cout << sumNabla_debug[r] << " ";
//		}
//		std::cout << std::endl;
//		std::cout << "*--------------------------------------------*" << std::endl;
//
//
//
//		std::cout << "-----------------CALCULATED---------------------------" << std::endl;
//		std::cout << noValidPoints << "   " << sumF << std::endl;
//
//
//		for (int r = 0, counter = 0; r < noPara; r++)
//		{
//			for (int c = 0; c <= r; c++, counter++)
//			{
//				std::cout << sumHessian[counter] << " ";
//			}
//		}
//		std::cout << std::endl;
//
//
//		for (int r = 0; r < noPara; r++)
//		{
//			std::cout << sumNabla[r] << " ";
//		}
//		std::cout << std::endl;
//		std::cout << "--------------------------------------------\n" << std::endl;
//
//		throw std::runtime_error("OpenCL ICP RMSE too high");
//	}
#endif



//	sdkStopTimer(&debug_timer);
//	float debug_time = sdkGetTimerValue(&debug_timer);
//	if(debug_time > 0.01){ std::cout << "DEBUG_TIME: " << debug_time << "ms" << std::endl; }


	return noValidPoints;
}
