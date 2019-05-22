/*
 * IntegrateIntoSceneData.h
 *
 *  Created on: Nov 13, 2016
 *      Author: qkgautier
 */

#ifndef INTEGRATEINTOSCENEDATA_H_
#define INTEGRATEINTOSCENEDATA_H_

#include <fstream>
#include <iostream>

#include "integrate.h"



namespace reconstruct_hls
{


template<class T, class SizeT> void resetArray(T*& array, SizeT size, const T* input)
{
	if(array){ delete[] array; }
	array = new T[(size_t)size];
	std::copy(input, input+size, array);
}

template<class T> void readValue(std::istream& stream, T& value)
{
	stream.read((char*)&value, sizeof(T));
}

template<class ArrayT, class SizeT> void readArray(std::istream& stream, ArrayT*& array, SizeT& size)
{
	readValue(stream, size);
	if(size > (SizeT)0)
	{
		if(array){ delete[] array; }
		array = new ArrayT[size];
		stream.read((char*)array, sizeof(ArrayT)*size);
	}
}

template<class T> void writeValue(std::ostream& stream, T& value)
{
	stream.write((const char*)&value, sizeof(T));
}

template<class ArrayT, class SizeT> void writeArray(std::ostream& stream, const ArrayT* array, SizeT& size)
{
	if(array && size > (SizeT)0)
	{
		writeValue(stream, size);
		stream.write((const char*)array, sizeof(ArrayT)*size);
	}
	else
	{
		SizeT zero = (SizeT)0;
		writeValue(stream, zero);
	}
}



class IntegrateIntoSceneData
{
public:

	IntegrateIntoSceneData():
		localVBA_(0),hashTable_(0),visibleEntryIds_(0),depth_(0),localVBAout_(0), pointsRay_(0){}

	~IntegrateIntoSceneData()
	{
		if(localVBA_){ delete[] localVBA_; }
		if(hashTable_){ delete[] hashTable_; }
		if(visibleEntryIds_){ delete[] visibleEntryIds_; }
		if(depth_){ delete[] depth_; }
		if(localVBAout_){ delete[] localVBAout_; }
		if(pointsRay_){ delete[] pointsRay_; }
	}


	void setArrays(
			ITMVoxel_s* localVBA, int allocatedSize,
			ITMHashEntry* hashTable, int noTotalEntries,
			int* visibleEntryIds, int noVisibleEntries,
			float* depth, int depthImgSizeX, int depthImgSizeY)
	{
		resetArray(localVBA_, allocatedSize, localVBA);
		allocatedSize_ = allocatedSize;

		resetArray(hashTable_, noTotalEntries, hashTable);
		noTotalEntries_ = noTotalEntries;

		resetArray(visibleEntryIds_, noVisibleEntries, visibleEntryIds);
		noVisibleEntries_ = noVisibleEntries;
		noTotalEntries_ = noTotalEntries;

		resetArray(depth_, depthImgSizeX*depthImgSizeY, depth);
		depthImgSize_ = depthImgSizeX*depthImgSizeY;
	}

	void setOutput(ITMVoxel_s* localVBAout, float4* pointsRay = 0)
	{
		resetArray(localVBAout_, allocatedSize_, localVBAout);

		if(pointsRay){ resetArray(pointsRay_, depthImgSize_, pointsRay); }
	}

	void writeInputs(const char* filename)
	{
		std::ofstream file(filename);
		if(!file.is_open()){ std::cerr << "Error" << std::endl; return; }

		writeValue(file, depthImgSize);
		writeValue(file, voxelSize);
		writeValue(file, M_d);
		writeValue(file, projParams_d);
		writeValue(file, mu);
		writeValue(file, maxW);
		writeValue(file, noVisibleEntries);
		writeValue(file, stopIntegratingAtMaxW);

		writeArray(file, localVBA_,        allocatedSize_);
		writeArray(file, hashTable_,       noTotalEntries_);
		writeArray(file, visibleEntryIds_, noVisibleEntries_);
		writeArray(file, depth_,           depthImgSize_);
	}

	void writeOutput(const char* filename)
	{
		std::ofstream file(filename);
		if(!file.is_open()){ std::cerr << "Error" << std::endl; return; }

		writeArray(file, localVBAout_, allocatedSize_);
		writeArray(file, pointsRay_,   depthImgSize_);
	}

	void readInputs(const char* filename)
	{
		std::ifstream file(filename);
		if(!file.is_open()){ std::cerr << "Error" << std::endl; return; }

		readValue(file, depthImgSize);
		readValue(file, voxelSize);
		readValue(file, M_d);
		readValue(file, projParams_d);
		readValue(file, mu);
		readValue(file, maxW);
		readValue(file, noVisibleEntries);
		readValue(file, stopIntegratingAtMaxW);

		readArray(file, localVBA_, allocatedSize_);
		readArray(file, hashTable_, noTotalEntries_);
		readArray(file, visibleEntryIds_, noVisibleEntries_);
		readArray(file, depth_, depthImgSize_);
	}

	void readOutput(const char* filename)
	{
		std::ifstream file(filename);
		if(!file.is_open()){ std::cerr << "Error" << std::endl; return; }

		readArray(file, localVBAout_, allocatedSize_);

		int pointsRaySize = 0;
		readArray(file, pointsRay_, pointsRaySize);
	}

	void print()
	{
		std::cout << "depthImgSize: " << depthImgSize.x << " " << depthImgSize.y << std::endl;
		std::cout << "voxelSize: " << voxelSize << std::endl;
		//Mat44  M_d;
		//float4 projParams_d;
		//float  mu;
		//int    maxW;
		//int    noVisibleEntries;
		//uchar  stopIntegratingAtMaxW;

		std::cout << "allocatedSize_: " << allocatedSize_ << std::endl;
		std::cout << "noTotalEntries_: " << noTotalEntries_ << std::endl;
		std::cout << "noVisibleEntries_: " << noVisibleEntries << " (" << noVisibleEntries_ << ")" << std::endl;
		std::cout << "depthImgSize_: " << depthImgSize_ << std::endl;
	}

public:
	int2 depthImgSize;
	float voxelSize;
	Mat44 M_d;
	float4 projParams_d;
	float mu;
	int maxW;
	int noVisibleEntries;
	uchar stopIntegratingAtMaxW;

	ITMVoxel_s* localVBA_;
	int allocatedSize_;
	ITMHashEntry* hashTable_;
	int noTotalEntries_;
	int* visibleEntryIds_;
	int noVisibleEntries_;
	float* depth_;
	int depthImgSize_;

	ITMVoxel_s* localVBAout_;
	float4* pointsRay_;
};

}// namespace

#endif /* INTEGRATEINTOSCENEDATA_H_ */
