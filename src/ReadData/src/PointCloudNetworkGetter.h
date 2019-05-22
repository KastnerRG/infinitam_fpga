/*
 * PointCloudNetworkGetter.h
 *
 *  Created on: Feb 14, 2017
 *      Author: qkgautier
 */

#ifndef SRC_POINTCLOUDNETWORKGETTER_H_
#define SRC_POINTCLOUDNETWORKGETTER_H_

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <mutex>
#include <condition_variable>

#include "TcpServer.h"

class PointCloudNetworkGetter: public TcpDataFunctor
{
public:
	PointCloudNetworkGetter(bool blocking = false);
	virtual ~PointCloudNetworkGetter();
	virtual void operator()(const char* data, size_t data_size);

	virtual void waitForData(pcl::PointCloud<pcl::PointXYZ>::Ptr ptr);


protected:
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_;
	std::mutex points_mutex;
	std::condition_variable cv_;
	bool data_ready_;
	bool blocking_;
};


static void pointCloudServer_task(
		short port,
		PointCloudNetworkGetter* getter,
		std::shared_ptr<TcpServer>& server)
{
	try
	{
		boost::asio::io_service io_service;
		server = std::make_shared<TcpServer>(io_service, port, getter);
		io_service.run();
	}
	catch(std::exception& e)
	{
		std::cerr << "Error while creating server: " << e.what() << std::endl;
	}
}


#endif /* SRC_POINTCLOUDNETWORKGETTER_H_ */
