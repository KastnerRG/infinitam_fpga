/*
 * PointCloudNetworkGetter.cpp
 *
 *  Created on: Feb 14, 2017
 *      Author: qkgautier
 */

#include "PointCloudNetworkGetter.h"

using namespace std;
using namespace pcl;

PointCloudNetworkGetter::PointCloudNetworkGetter(bool blocking):
		data_ready_(false),
		blocking_(blocking)
{}

PointCloudNetworkGetter::~PointCloudNetworkGetter(){}

void PointCloudNetworkGetter::operator()(const char* data, size_t data_size)
{
	if(data_ready_ && !blocking_){ return; }

	unique_lock<mutex> lock(points_mutex);
	cv_.wait(lock, [&]{return ptr_;}); // !data_ready_ &&

	const float* points = reinterpret_cast<const float*>(data);
	size_t num_points = data_size / sizeof(float) / 3;
	ptr_->resize(num_points);
	for(size_t i = 0; i < num_points; i++)
	{
		(*ptr_)[i].x = points[i*3+0];
		(*ptr_)[i].y = points[i*3+1];
		(*ptr_)[i].z = points[i*3+2];
	}

	data_ready_ = true;
	lock.unlock();
	cv_.notify_all();
}

void PointCloudNetworkGetter::waitForData(PointCloud<PointXYZ>::Ptr ptr)
{
	{
		lock_guard<mutex> lock(points_mutex);
		ptr_ = ptr;
		data_ready_ = false;
	}
	cv_.notify_all();


	unique_lock<mutex> lock(points_mutex);
	cv_.wait(lock, [&]{return data_ready_;});

	ptr_.reset();

	lock.unlock();
	cv_.notify_all();
}
