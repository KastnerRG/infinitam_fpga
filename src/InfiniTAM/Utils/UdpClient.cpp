/*
 * UdpClient.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: qkgautier
 */

#include "UdpClient.h"

#include <boost/bind.hpp>

using namespace boost;
using boost::asio::ip::udp;
using namespace std;


UdpClient::UdpClient(
		asio::io_service& io_service,
		const string& host,
		const string& port):
				io_service_(io_service),
				socket_(io_service, udp::endpoint(udp::v4(), 0))
{
	udp::resolver resolver(io_service_);
	udp::resolver::query query(udp::v4(), host, port);
	endpoint_ = *resolver.resolve(query);
}

UdpClient::~UdpClient()
{
	socket_.close();
}


void UdpClient::send(const char* data, size_t data_size)
{
	socket_.send_to(asio::buffer(data, data_size), endpoint_);
}
void UdpClient::send(const vector<char>& data){ send(data.data(), data.size()); }
void UdpClient::send(const string& msg)       { send(msg.c_str(), msg.size()); }

void UdpClient::asyncSend(const char* data, size_t data_size)
{
	socket_.async_send_to(boost::asio::buffer(data, data_size), endpoint_,
	          boost::bind(&UdpClient::handle_send, this,
	            boost::asio::placeholders::error,
	            boost::asio::placeholders::bytes_transferred));
}
void UdpClient::asyncSend(const vector<char>& data){ asyncSend(data.data(), data.size()); }
void UdpClient::asyncSend(const string& msg)       { asyncSend(msg.c_str(), msg.size()); }

void UdpClient::sendLargeBuffer(
		const char* buffer,
		size_t buffer_size,
		uint32_t buffer_idx,
		size_t step_size)
{
	size_t chunk_idx_size  = sizeof(uint32_t);
	size_t buffer_idx_size = sizeof(uint32_t);

	vector<char> chunk(buffer_idx_size + chunk_idx_size + step_size, 0);

	for(uint32_t i = 0; i < buffer_size; i+=step_size)
	{
		size_t len = std::min(step_size, buffer_size - 1);

		std::copy(&buffer[i], &buffer[i] + len, &chunk[buffer_idx_size+chunk_idx_size]); // chunk data
		*(uint32_t*)(&chunk[0]) = buffer_idx;      // buffer index
		*(uint32_t*)(&chunk[buffer_idx_size]) = i; // chunk index

		send(chunk.data(), chunk.size());
	}
}






