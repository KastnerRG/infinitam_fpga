/*
 * UdpClient.h
 *
 *  Created on: Feb 9, 2017
 *      Author: qkgautier
 */

#pragma once

#include <boost/array.hpp>
#include <boost/asio.hpp>

class UdpClient
{
public:
	UdpClient(
			boost::asio::io_service& io_service,
			const std::string& host,
			const std::string& port);

	virtual ~UdpClient();

	void send(const char* data, size_t data_size);
	void send(const std::vector<char>& data);
	void send(const std::string& msg);

	void asyncSend(const char* data, size_t data_size);
	void asyncSend(const std::vector<char>& data);
	void asyncSend(const std::string& msg);

	/**
	 * Synchronously send a large buffer
	 * by dividing it into multiple chunks: [buffer_idx chunk_idx chunk_data]
	 * Note that chunk size = step_size + sizeof(chunk_idx) + sizeof(buffer_idx)
	 */
	void sendLargeBuffer(
			const char* buffer,
			size_t buffer_size,
			uint32_t buffer_idx,
			size_t step_size = 1024);

private:
	boost::asio::io_service& io_service_;
	boost::asio::ip::udp::socket socket_;
	boost::asio::ip::udp::endpoint endpoint_;

	void handle_send(
			const boost::system::error_code& error,
			std::size_t bytes_transferred){}
 };

