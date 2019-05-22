// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMVisualisationEngine_OpenCL.h"
#include "../../DeviceAgnostic/ITMRepresentationAccess.h"
#include "../../DeviceAgnostic/ITMVisualisationEngine.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

#include "../../../../ORUtils/OpenCLContext.h"

#include "ITMSceneReconstructionEngine_OpenCL_knobs.h"

#include <vector>

using namespace ITMLib::Engine;

template<class TVoxel, class TIndex>
static int RenderPointCloud(Vector4u *outRendering, Vector4f *locations, Vector4f *colours, const Vector4f *ptsRay, 
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, bool skipPoints, float voxelSize, 
	Vector2i imgSize, Vector3f lightSource);

template<class TVoxel, class TIndex>
ITMRenderState* ITMVisualisationEngine_OpenCL<TVoxel, TIndex>::CreateRenderState(const Vector2i & imgSize) const
{
	return new ITMRenderState(
		imgSize, this->scene->sceneParams->viewFrustum_min, this->scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CPU
	);
}

template<class TVoxel>
ITMRenderState_VH* ITMVisualisationEngine_OpenCL<TVoxel, ITMVoxelBlockHash>::CreateRenderState(const Vector2i & imgSize) const
{
	return new ITMRenderState_VH(
		ITMVoxelBlockHash::noTotalEntries, imgSize, this->scene->sceneParams->viewFrustum_min, this->scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CPU
	);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_OpenCL<TVoxel, TIndex>::FindVisibleBlocks(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const
{
}

template<class TVoxel>
void ITMVisualisationEngine_OpenCL<TVoxel,ITMVoxelBlockHash>::FindVisibleBlocks(const ITMPose *pose, const ITMIntrinsics *intrinsics,
	ITMRenderState *renderState) const
{
	const ITMHashEntry *hashTable = this->scene->index.GetEntries();
	int noTotalEntries = this->scene->index.noTotalEntries;
	float voxelSize = this->scene->sceneParams->voxelSize;
	Vector2i imgSize = renderState->renderingRangeImage->noDims;

	Matrix4f M = pose->GetM();
	Vector4f projParams = intrinsics->projectionParamsSimple.all;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	int noVisibleEntries = 0;
	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();

	//build visible list
	for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
	{
		unsigned char hashVisibleType = 0;// = entriesVisibleType[targetIdx];
		const ITMHashEntry &hashEntry = hashTable[targetIdx];

		if (hashEntry.ptr >= 0)
		{
			bool isVisible, isVisibleEnlarged;
			checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M, projParams, voxelSize, imgSize);
			hashVisibleType = isVisible;
		}

		if (hashVisibleType > 0)
		{
			visibleEntryIDs[noVisibleEntries] = targetIdx;
			noVisibleEntries++;
		}
	}

	renderState_vh->noVisibleEntries = noVisibleEntries;
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_OpenCL<TVoxel, TIndex>::CreateExpectedDepths(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const
{
	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId) {
		//TODO : this could be improved a bit...
		Vector2f & pixel = minmaxData[locId];
		pixel.x = 0.2f;
		pixel.y = 3.0f;
	}
}

template<class TVoxel>
void ITMVisualisationEngine_OpenCL<TVoxel,ITMVoxelBlockHash>::CreateExpectedDepths(const ITMPose *pose, const ITMIntrinsics *intrinsics,
	ITMRenderState *renderState) const
{
	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId) {
		Vector2f & pixel = minmaxData[locId];
		pixel.x = FAR_AWAY;
		pixel.y = VERY_CLOSE;
	}

	float voxelSize = this->scene->sceneParams->voxelSize;

	std::vector<RenderingBlock> renderingBlocks(MAX_RENDERING_BLOCKS);
	int numRenderingBlocks = 0;

	ITMRenderState_VH* renderState_vh = (ITMRenderState_VH*)renderState;

	const int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	int noVisibleEntries = renderState_vh->noVisibleEntries;

	//go through list of visible 8x8x8 blocks
	for (int blockNo = 0; blockNo < noVisibleEntries; ++blockNo) {
		const ITMHashEntry & blockData(this->scene->index.GetEntries()[visibleEntryIDs[blockNo]]);

		Vector2i upperLeft, lowerRight;
		Vector2f zRange;
		bool validProjection = false;
		if (blockData.ptr>=0) {
			validProjection = ProjectSingleBlock(blockData.pos, pose->GetM(), intrinsics->projectionParamsSimple.all, imgSize, voxelSize, upperLeft, lowerRight, zRange);
		}
		if (!validProjection) continue;

		Vector2i requiredRenderingBlocks((int)ceilf((float)(lowerRight.x - upperLeft.x + 1) / (float)renderingBlockSizeX), 
			(int)ceilf((float)(lowerRight.y - upperLeft.y + 1) / (float)renderingBlockSizeY));
		int requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;

		if (numRenderingBlocks + requiredNumBlocks >= MAX_RENDERING_BLOCKS) continue;
		int offset = numRenderingBlocks;
		numRenderingBlocks += requiredNumBlocks;

		CreateRenderingBlocks(&(renderingBlocks[0]), offset, upperLeft, lowerRight, zRange);
	}

	// go through rendering blocks
	for (int blockNo = 0; blockNo < numRenderingBlocks; ++blockNo) {
		// fill minmaxData
		const RenderingBlock & b(renderingBlocks[blockNo]);

		for (int y = b.upperLeft.y; y <= b.lowerRight.y; ++y) {
			for (int x = b.upperLeft.x; x <= b.lowerRight.x; ++x) {
				Vector2f & pixel(minmaxData[x + y*imgSize.x]);
				if (pixel.x > b.zRange.x) pixel.x = b.zRange.x;
				if (pixel.y < b.zRange.y) pixel.y = b.zRange.y;
			}
		}
	}
}

template<class TVoxel, class TIndex>
static void GenericRaycast(const ITMScene<TVoxel,TIndex> *scene, const Vector2i& imgSize, const Matrix4f& invM, Vector4f projParams, const ITMRenderState *renderState)
{
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	float mu = scene->sceneParams->mu;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId)
	{
		int y = locId/imgSize.x;
		int x = locId - y*imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		castRay<TVoxel, TIndex>(
			pointsRay[locId],
			x, y,
			voxelData,
			voxelIndex,
			invM,
			projParams,
			oneOverVoxelSize,
			mu,
			minmaximg[locId2]
		);
	}
}

template<class TVoxel, class TIndex>
static void RenderImage_common(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	const ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type)
{
	Vector2i imgSize = outputImage->noDims;
	Matrix4f invM = pose->GetInvM();

	GenericRaycast(scene, imgSize, invM, intrinsics->projectionParamsSimple.all, renderState);

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

	if ((type == IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME)&&
	    (!TVoxel::hasColorInformation)) type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;

	switch (type) {
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
		break;
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelNormal<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
		break;
	case IITMVisualisationEngine::RENDER_SHADED_GREYSCALE:
	default:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
	}
}

template<class TVoxel, class TIndex>
static void CreatePointCloud_common(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState, bool skipPoints)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f invM = trackingState->pose_d->GetInvM() * view->calib->trafo_rgb_to_depth.calib;

	GenericRaycast(scene, imgSize, invM, view->calib->intrinsics_rgb.projectionParamsSimple.all, renderState);
	trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

	trackingState->pointCloud->noTotalPoints = RenderPointCloud<TVoxel, TIndex>(
		renderState->raycastImage->GetData(MEMORYDEVICE_CPU),
		trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU),
		trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU),
		renderState->raycastResult->GetData(MEMORYDEVICE_CPU),
		scene->localVBA.GetVoxelBlocks(),
		scene->index.getIndexData(),
		skipPoints,
		scene->sceneParams->voxelSize,
		imgSize,
		-Vector3f(invM.getColumn(2))
	);
}

template<class TVoxel, class TIndex>
static void CreateICPMaps_common(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f invM = trackingState->pose_d->GetInvM();

	if(!renderState->rayCastInSceneIntegration)
	{
		GenericRaycast(scene, imgSize, invM, view->calib->intrinsics_d.projectionParamsSimple.all, renderState);
	}
	trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);
	Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	float voxelSize = scene->sceneParams->voxelSize;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		processPixelICP<true>(outRendering, pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
}

template<class TVoxel, class TIndex>
static void ForwardRender_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f M = trackingState->pose_d->GetM();
	Matrix4f invM = trackingState->pose_d->GetInvM();
	Vector4f projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	Vector4f invProjParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	invProjParams.x = 1.0f / invProjParams.x;
	invProjParams.y = 1.0f / invProjParams.y;

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	const Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	Vector4f *forwardProjection = renderState->forwardProjection->GetData(MEMORYDEVICE_CPU);
	float *currentDepth = view->depth->GetData(MEMORYDEVICE_CPU);
	int *fwdProjMissingPoints = renderState->fwdProjMissingPoints->GetData(MEMORYDEVICE_CPU);
	Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CPU);
	const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	float voxelSize = scene->sceneParams->voxelSize;
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

	renderState->forwardProjection->Clear();

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	{
		int locId = x + y * imgSize.x;
		Vector4f pixel = pointsRay[locId];

		int locId_new = forwardProjectPixel(pixel * voxelSize, M, projParams, imgSize);
		if (locId_new >= 0) forwardProjection[locId_new] = pixel;
	}

	int noMissingPoints = 0;
	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	{
		int locId = x + y * imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		Vector4f fwdPoint = forwardProjection[locId];
		Vector2f minmaxval = minmaximg[locId2];
		float depth = currentDepth[locId];

		if ((fwdPoint.w <= 0) && ((fwdPoint.x == 0 && fwdPoint.y == 0 && fwdPoint.z == 0) || (depth >= 0)) && (minmaxval.x < minmaxval.y))
		//if ((fwdPoint.w <= 0) && (minmaxval.x < minmaxval.y))
		{
			fwdProjMissingPoints[noMissingPoints] = locId;
			noMissingPoints++;
		}
	}

	renderState->noFwdProjMissingPoints = noMissingPoints;
    
	for (int pointId = 0; pointId < noMissingPoints; pointId++)
	{
		int locId = fwdProjMissingPoints[pointId];
		int y = locId / imgSize.x, x = locId - y*imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		castRay<TVoxel, TIndex>(forwardProjection[locId], x, y, voxelData, voxelIndex, invM, invProjParams,
			1.0f / scene->sceneParams->voxelSize, scene->sceneParams->mu, minmaximg[locId2]);
	}

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		processPixelForwardRender<true>(outRendering, forwardProjection, imgSize, x, y, voxelSize, lightSource);
}


template<class TVoxel>
ITMVisualisationEngine_OpenCL<TVoxel,ITMVoxelBlockHash>::~ITMVisualisationEngine_OpenCL(void)
{
	if(invM_device_){ clReleaseMemObject(invM_device_); }
#ifdef OPENCL_DEBUG
	if(pointsRay_device_){ clReleaseMemObject(pointsRay_device_); }
	if(voxelData_device_){ clReleaseMemObject(voxelData_device_); }
	if(voxelIndex_device_){ clReleaseMemObject(voxelIndex_device_); }
	if(minmaximg_device_){ clReleaseMemObject(minmaximg_device_); }
#endif
}



template<class TVoxel, class TIndex>
void ITMVisualisationEngine_OpenCL<TVoxel,TIndex>::RenderImage(const ITMPose *pose, const ITMIntrinsics *intrinsics,
	const ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type) const
{
	RenderImage_common(this->scene, pose, intrinsics, renderState, outputImage, type);
}

template<class TVoxel>
void ITMVisualisationEngine_OpenCL<TVoxel,ITMVoxelBlockHash>::RenderImage(const ITMPose *pose,  const ITMIntrinsics *intrinsics,
	const ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type) const
{
	RenderImage_common(this->scene, pose, intrinsics, renderState, outputImage, type);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_OpenCL<TVoxel, TIndex>::FindSurface(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const
{
	GenericRaycast(this->scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_OpenCL<TVoxel,ITMVoxelBlockHash>::FindSurface(const ITMPose *pose, const ITMIntrinsics *intrinsics,
	const ITMRenderState *renderState) const
{
	GenericRaycast(this->scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all, renderState);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_OpenCL<TVoxel,TIndex>::CreatePointCloud(const ITMView *view, ITMTrackingState *trackingState,
	ITMRenderState *renderState, bool skipPoints) const
{ 
	CreatePointCloud_common(this->scene, view, trackingState, renderState, skipPoints);
}

template<class TVoxel>
void ITMVisualisationEngine_OpenCL<TVoxel,ITMVoxelBlockHash>::CreatePointCloud(const ITMView *view, ITMTrackingState *trackingState,
	ITMRenderState *renderState, bool skipPoints) const
{
	CreatePointCloud_common(this->scene, view, trackingState, renderState, skipPoints);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_OpenCL<TVoxel,TIndex>::CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const
{
	CreateICPMaps_common(this->scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_OpenCL<TVoxel,ITMVoxelBlockHash>::CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState,
	ITMRenderState *renderState) const
{
	CreateICPMaps_common(this->scene, view, trackingState, renderState);
}

#ifdef COMPILE_WITH_OPENCL
#include "../../../../Utils/NVTimer.h"
// Have to put namespace here because of a bug in some versions of GCC
namespace ITMLib
{
	namespace Engine
	{
		typedef struct
		{
			cl_float4 data0;
			cl_float4 data1;
			cl_float4 data2;
			cl_float4 data3;

		} Mat44_cl;

//		inline ITMVoxel_s readVoxel_s(const ITMVoxel_s *voxelData, const ITMLib::Objects::ITMVoxelBlockHash::IndexData *voxelIndex,
//			const Vector3i & point, bool &isFound, ITMLib::Objects::ITMVoxelBlockHash::IndexCache & cache)
//		{
//			Vector3i blockPos;
//
//			blockPos.x = ((point.x < 0) ? point.x - SDF_BLOCK_SIZE + 1 : point.x) / SDF_BLOCK_SIZE;
//			blockPos.y = ((point.y < 0) ? point.y - SDF_BLOCK_SIZE + 1 : point.y) / SDF_BLOCK_SIZE;
//			blockPos.z = ((point.z < 0) ? point.z - SDF_BLOCK_SIZE + 1 : point.z) / SDF_BLOCK_SIZE;
//
//			int linearIdx = point.x + (point.y - blockPos.x) * SDF_BLOCK_SIZE + (point.z - blockPos.y) * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE - blockPos.z * SDF_BLOCK_SIZE3;
//
//			if IS_EQUAL3(blockPos, cache.blockPos)
//			{
//				isFound = true;
//				return voxelData[cache.blockPtr + linearIdx];
//			}
//
//			int hashIdx = hashIndex(blockPos);
//
//			while (true)
//			{
//				ITMHashEntry hashEntry = voxelIndex[hashIdx];
//
//				if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0)
//				{
//					isFound = true;
//					cache.blockPos = blockPos; cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
//					return voxelData[cache.blockPtr + linearIdx];
//				}
//
//				if (hashEntry.offset < 1) break;
//				hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
//			}
//
//			isFound = false;
//			return ITMVoxel_s();
//		}
//
//		inline float readFromSDF_float_uninterpolated_s(const ITMVoxel_s *voxelData,
//			const ITMHashEntry *voxelIndex, Vector3f point, bool &isFound, ITMLib::Objects::ITMVoxelBlockHash::IndexCache & cache)
//		{
//			ITMVoxel_s res = readVoxel_s(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)), isFound, cache);
//			return ITMVoxel_s::SDF_valueToFloat(res.sdf);
//		}
//
//		inline float readFromSDF_float_interpolated_s(const ITMVoxel_s *voxelData,
//			const ITMHashEntry *voxelIndex, Vector3f point, bool &isFound, ITMLib::Objects::ITMVoxelBlockHash::IndexCache & cache)
//		{
//			float res1, res2, v1, v2;
//			Vector3f coeff; Vector3i pos;
//
//			//pos = point.toIntFloor(coeff);
//			Vector3f intFloor(floor(point.x), floor(point.y), floor(point.z));
//			coeff = point - intFloor;
//			pos = Vector3i((int)intFloor.x, (int)intFloor.y, (int)intFloor.z);
//
//
//			v1 = readVoxel_s(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), isFound, cache).sdf;
//			v2 = readVoxel_s(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), isFound, cache).sdf;
//			res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;
//
//			v1 = readVoxel_s(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), isFound, cache).sdf;
//			v2 = readVoxel_s(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), isFound, cache).sdf;
//			res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
//
//			v1 = readVoxel_s(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), isFound, cache).sdf;
//			v2 = readVoxel_s(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), isFound, cache).sdf;
//			res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;
//
//			v1 = readVoxel_s(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), isFound, cache).sdf;
//			v2 = readVoxel_s(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), isFound, cache).sdf;
//			res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
//
//			isFound = true;
//			return ITMVoxel_s::SDF_valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
//		}

		template<>
		void ITMVisualisationEngine_OpenCL<ITMVoxel_s,ITMVoxelBlockHash>::CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState,
			ITMRenderState *renderState) const
		{
			ORUtils::OpenCLContext::getInstance().programAocxKernel(ORUtils::OpenCLContext::KERNEL_RAYCAST);

			ORUtils::OpenCLContext::ClContext& clContext = ORUtils::OpenCLContext::getInstance().cl_context_;



			Vector2i imgSize = renderState->raycastResult->noDims;
			Matrix4f invM = trackingState->pose_d->GetInvM();


			// -----------------------
			// GenericRaycast
			// -----------------------

			//GenericRaycast(this->scene, imgSize, invM, view->calib->intrinsics_d.projectionParamsSimple.all, renderState);


			Vector4f projParams = view->calib->intrinsics_d.projectionParamsSimple.all;

			projParams.x = 1.0f / projParams.x;
			projParams.y = 1.0f / projParams.y;

			float mu               = this->scene->sceneParams->mu;
			float oneOverVoxelSize = 1.0f / this->scene->sceneParams->voxelSize;
			float stepScale        = mu * oneOverVoxelSize;

			// CPU memory pointers
#ifdef OPENCL_DEBUG
			const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
			const ITMVoxel_s *voxelData = this->scene->localVBA.GetVoxelBlocks();
			const ITMHashEntry *voxelIndex = this->scene->index.getIndexData();
#endif
			Vector4f *pointsRay    = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);


			// OpenCL

			Matrix4f invM_t = invM.t(); // VERY IMPORTANT: Matrix4f is stored columnwise! So we use the transpose in OpenCL.

			int err;
			int qid = ORUtils::OpenCLContext::KERNEL_RAYCAST;
			int kid = ORUtils::OpenCLContext::KERNEL_RAYCAST;


#ifdef OPENCL_DEBUG
			if(!pointsRay_device_)
			{
				cl_mem* pointsRay_device = const_cast<cl_mem*>(&pointsRay_device_);
				*pointsRay_device = clCreateBuffer(clContext.context, CL_MEM_READ_WRITE, sizeof(cl_float4) * imgSize.x * imgSize.y, NULL, &err);
				if(!clCheckErr(err, "Failed to allocate memory!")){ return; }
			}

			if(!ORUtils::OpenCLContext::getInstance().voxelData_cl)
			{
				if(!voxelData_device_)
				{
					cl_mem* voxelData_device = const_cast<cl_mem*>(&voxelData_device_);
					*voxelData_device = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(ITMVoxel_s) * this->scene->localVBA.allocatedSize, NULL, &err);
					if(!clCheckErr(err, "Failed to allocate memory!")){ return; }
				}

				err = clEnqueueWriteBuffer(clContext.queues[qid], voxelData_device_, CL_FALSE, 0, sizeof(ITMVoxel_s) * this->scene->localVBA.allocatedSize, (void*)voxelData, 0, NULL, NULL);
				if(!clCheckErr(err, "Failed to write data to the device!")){ return; }
			}
			else
			{
				cl_mem* voxelData_device = const_cast<cl_mem*>(&voxelData_device_);
				*voxelData_device = ORUtils::OpenCLContext::getInstance().voxelData_cl;
			}

			if(!voxelIndex_device_)
			{
				cl_mem* voxelIndex_device = const_cast<cl_mem*>(&voxelIndex_device_);
				*voxelIndex_device = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(ITMHashEntry) * this->scene->index.noTotalEntries, NULL, &err);
				if(!clCheckErr(err, "Failed to allocate memory!")){ return; }
			}

			if(!minmaximg_device_)
			{
				cl_mem* minmaximg_device = const_cast<cl_mem*>(&minmaximg_device_);
				*minmaximg_device = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(Vector2f) * renderState->renderingRangeImage->dataSize, NULL, &err);
				if(!clCheckErr(err, "Failed to allocate memory!")){ return; }
			}

			err = clEnqueueWriteBuffer(clContext.queues[qid], pointsRay_device_, CL_FALSE, 0, sizeof(cl_float4) * imgSize.x * imgSize.y, (void*)pointsRay, 0, NULL, NULL);
			if(!clCheckErr(err, "Failed to write data to the device!")){ return; }

			err = clEnqueueWriteBuffer(clContext.queues[qid], voxelIndex_device_, CL_FALSE, 0, sizeof(ITMHashEntry) * this->scene->index.noTotalEntries, (void*)voxelIndex, 0, NULL, NULL);
			if(!clCheckErr(err, "Failed to write data to the device!")){ return; }

			err = clEnqueueWriteBuffer(clContext.queues[qid], minmaximg_device_, CL_FALSE, 0, sizeof(Vector2f) * renderState->renderingRangeImage->dataSize, (void*)minmaximg, 0, NULL, NULL);
			if(!clCheckErr(err, "Failed to write data to the device!")){ return; }

			cl_mem pointsRay_device     = pointsRay_device_;
			cl_mem voxelData_device     = voxelData_device_;
			cl_mem voxelIndex_device    = voxelIndex_device_;
			cl_mem minmaximg_device     = minmaximg_device_;

#else
			cl_mem pointsRay_device     = renderState->raycastResult->GetOpenCLData();
			cl_mem voxelData_device     = this->scene->localVBA.GetVoxelBlocks_cl();
			cl_mem voxelIndex_device    = this->scene->index.getIndexData_cl();
			cl_mem minmaximg_device     = renderState->renderingRangeImage->GetOpenCLData();
#endif

			cl_int2   imgSize_cl    = *reinterpret_cast<cl_int2*>(&imgSize);
			cl_float4 projParams_cl = *reinterpret_cast<cl_float4*>(&projParams);
			Mat44_cl  invM_cl       = *reinterpret_cast<Mat44_cl*>(&invM_t);

			if(!invM_device_)
			{
				cl_mem* invM_device = const_cast<cl_mem*>(&invM_device_);
				*invM_device = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(Mat44_cl), NULL, &err);
				if(!clCheckErr(err, "Failed to allocate memory!")){ *invM_device = 0; return; }
			}
//			cl_mem invM_device = clCreateBuffer(clContext.context, CL_MEM_READ_ONLY, sizeof(Mat44_cl), NULL, &err);
//			if(!clCheckErr(err, "Failed to allocate memory!")){ return; }

			err = clEnqueueWriteBuffer(clContext.queues[qid], invM_device_, CL_FALSE, 0, sizeof(Mat44_cl), (void*)&invM_cl, 0, NULL, NULL);
			if(!clCheckErr(err, "Failed to write data to the device!")){ return; }



			size_t localWorkSize[3] = {WORK_GROUP_SIZE_X2, WORK_GROUP_SIZE_Y2, WORK_GROUP_SIZE_Z2};

#if KNOB2_USE_ND_RANGE
			if( (clContext.device_type & CL_DEVICE_TYPE_ACCELERATOR) == 0)
			{
				localWorkSize[0] = WORK_GROUP_SIZE_X2_NONFPGA;
				localWorkSize[1] = WORK_GROUP_SIZE_Y2_NONFPGA;
			}

			size_t workSize[3]      = {
					(size_t)(ceil((float)imgSize.x / localWorkSize[0]) * localWorkSize[0]),
					(size_t)(ceil((float)imgSize.y / localWorkSize[1]) * localWorkSize[1]),
					1};
#else
			size_t workSize[3]      = {localWorkSize[0], localWorkSize[1], localWorkSize[2]};
#endif

			int argid = 0;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem), (void*)&pointsRay_device); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem), (void*)&voxelData_device); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem), (void*)&voxelIndex_device); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem), (void*)&minmaximg_device); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_int2), (void*)&imgSize_cl); argid++;
			//clSetKernelArg(clContext.kernels[kid], argid, sizeof(float), (void*)&mu); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(float), (void*)&oneOverVoxelSize); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(float), (void*)&stepScale); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_float4), (void*)&projParams_cl); argid++;
			clSetKernelArg(clContext.kernels[kid], argid, sizeof(cl_mem), (void*)&invM_device_); argid++;

			err = clEnqueueNDRangeKernel(
					clContext.queues[qid], clContext.kernels[kid], 3, NULL,
					workSize, localWorkSize, 0, NULL, &clContext.events[0]);
#ifndef OPENCL_DEBUG
			clFinish(clContext.queues[qid]);
#endif
			if(!clCheckErr(err, "Failed to execute kernel!")){ return; }


#ifdef OPENCL_DEBUG
			err = clEnqueueReadBuffer(
					clContext.queues[qid], pointsRay_device_, CL_TRUE, 0,
					sizeof(cl_float4) * imgSize.x * imgSize.y, (void*)pointsRay, 0, NULL, NULL);
			if(!clCheckErr(err, "Failed to read data from the device!")){ return; }


			// ----
			// Kernel profiling with OpenCL
			//-----
			cl_ulong k_time_start, k_time_end;
			clGetEventProfilingInfo(clContext.events[0], CL_PROFILING_COMMAND_START, sizeof(k_time_start), &k_time_start, NULL);
			clGetEventProfilingInfo(clContext.events[0], CL_PROFILING_COMMAND_END, sizeof(k_time_end), &k_time_end, NULL);
			double kernel_time = (k_time_end - k_time_start) / 1000000.0;
			std::cout << "Raycast_CreateICPMaps_Kernel: " << kernel_time << "ms\n";
#endif
//			clReleaseMemObject(invM_device);





//		#ifdef WITH_OPENMP
//			#pragma omp parallel for
//		#endif
//			for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId)
//			{
//				int y = locId/imgSize.x;
//				int x = locId - y*imgSize.x;
//				int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;
//
//				// -------
//				// castRay
//				// -------
//
////				castRay<ITMVoxel_s, ITMVoxelBlockHash>(
////					pointsRay[locId],
////					x, y,
////					voxelData,
////					voxelIndex,
////					invM,
////					projParams,
////					oneOverVoxelSize,
////					mu,
////					minmaximg[locId2]
////				);
//
//
//				Vector4f& pt_out = pointsRay[locId];
//				Vector2f  viewFrustum_minmax = minmaximg[locId2];
//
//
//				Vector4f pt_camera_f; Vector3f pt_block_s, pt_block_e, rayDirection, pt_result;
//				bool pt_found, hash_found;
//				float sdfValue = 1.0f;
//				float totalLength, stepLength, totalLengthMax, stepScale;
//
//				stepScale = mu * oneOverVoxelSize;
//
//				pt_camera_f.z = viewFrustum_minmax.x;
//				pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams.z) * projParams.x);
//				pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams.w) * projParams.y);
//				pt_camera_f.w = 1.0f;
//				totalLength = length(TO_VECTOR3(pt_camera_f)) * oneOverVoxelSize;
//				pt_block_s = TO_VECTOR3(invM * pt_camera_f) * oneOverVoxelSize;
//
//				pt_camera_f.z = viewFrustum_minmax.y;
//				pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams.z) * projParams.x);
//				pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams.w) * projParams.y);
//				pt_camera_f.w = 1.0f;
//				totalLengthMax = length(TO_VECTOR3(pt_camera_f)) * oneOverVoxelSize;
//				pt_block_e = TO_VECTOR3(invM * pt_camera_f) * oneOverVoxelSize;
//
//				rayDirection = pt_block_e - pt_block_s;
//				float direction_norm = 1.0f / sqrt(rayDirection.x * rayDirection.x + rayDirection.y * rayDirection.y + rayDirection.z * rayDirection.z);
//				rayDirection *= direction_norm;
//
//				pt_result = pt_block_s;
//
//				ITMVoxelBlockHash::IndexCache cache;
//
//				while (totalLength < totalLengthMax)
//				{
//					sdfValue = readFromSDF_float_uninterpolated_s(voxelData, voxelIndex, pt_result, hash_found, cache);
//
//					if (!hash_found) { stepLength = SDF_BLOCK_SIZE; }
//					else
//					{
//						if ((sdfValue <= 0.1f) && (sdfValue >= -0.5f))
//						{
//							sdfValue = readFromSDF_float_interpolated_s(voxelData, voxelIndex, pt_result, hash_found, cache);
//						}
//						if (sdfValue <= 0.0f) break;
//						stepLength = MAX(sdfValue * stepScale, 1.0f);
//					}
//
//					pt_result += stepLength * rayDirection; totalLength += stepLength;
//				}
//
//				if (sdfValue <= 0.0f)
//				{
//					stepLength = sdfValue * stepScale;
//					pt_result += stepLength * rayDirection;
//
//					sdfValue = readFromSDF_float_interpolated_s(voxelData, voxelIndex, pt_result, hash_found, cache);
//					stepLength = sdfValue * stepScale;
//					pt_result += stepLength * rayDirection;
//
//					pt_found = true;
//				}
//				else pt_found = false;
//
//				pt_out.x = pt_result.x; pt_out.y = pt_result.y; pt_out.z = pt_result.z;
//				if (pt_found) pt_out.w = 1.0f; else pt_out.w = 0.0f;
//
//				// -------
//
//			}


			// -----------------------




			trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

			Vector3f lightSource = -Vector3f(invM.getColumn(2));
			Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);
			Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CPU);
			Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);
			//Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
			float voxelSize = scene->sceneParams->voxelSize;

		#ifdef WITH_OPENMP
			#pragma omp parallel for
		#endif
			for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
				processPixelICP<true>(outRendering, pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
		}
	}
}
#endif // COMPILE_WITH_OPENCL

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_OpenCL<TVoxel, TIndex>::ForwardRender(const ITMView *view, ITMTrackingState *trackingState,
	ITMRenderState *renderState) const
{
	ForwardRender_common(this->scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_OpenCL<TVoxel, ITMVoxelBlockHash>::ForwardRender(const ITMView *view, ITMTrackingState *trackingState,
	ITMRenderState *renderState) const
{
	ForwardRender_common(this->scene, view, trackingState, renderState);
}

template<class TVoxel, class TIndex>
static int RenderPointCloud(Vector4u *outRendering, Vector4f *locations, Vector4f *colours, const Vector4f *ptsRay, 
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, bool skipPoints, float voxelSize, 
	Vector2i imgSize, Vector3f lightSource)
{
	int noTotalPoints = 0;

	for (int y = 0, locId = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++, locId++)
	{
		Vector3f outNormal; float angle; 
		Vector4f pointRay = ptsRay[locId];
		Vector3f point = pointRay.toVector3();
		bool foundPoint = pointRay.w > 0;

		computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

		if (foundPoint) drawPixelGrey(outRendering[locId], angle);
		else outRendering[locId] = Vector4u((uchar)0);

		if (skipPoints && ((x % 2 == 0) || (y % 2 == 0))) foundPoint = false;

		if (foundPoint)
		{
			Vector4f tmp;
			tmp = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelData, voxelIndex, point);
			if (tmp.w > 0.0f) { tmp.x /= tmp.w; tmp.y /= tmp.w; tmp.z /= tmp.w; tmp.w = 1.0f; }
			colours[noTotalPoints] = tmp;

			Vector4f pt_ray_out;
			pt_ray_out.x = point.x * voxelSize; pt_ray_out.y = point.y * voxelSize;
			pt_ray_out.z = point.z * voxelSize; pt_ray_out.w = 1.0f;
			locations[noTotalPoints] = pt_ray_out;

			noTotalPoints++;
		}
	}

	return noTotalPoints;
}

template class ITMLib::Engine::ITMVisualisationEngine_OpenCL<ITMVoxel, ITMVoxelIndex>;
