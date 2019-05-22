/*
 * TcpServer.cpp
 *
 *  Created on: Feb 14, 2017
 *      Author: qkgautier
 */

#include "TcpServer.h"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>

#include <boost/bind.hpp>

using boost::asio::ip::tcp;

TcpSession::TcpSession(tcp::socket socket, TcpDataFunctor* data_functor):
	socket_(std::move(socket)),
	process_data_(data_functor)
{
	header_buffer_.resize(sizeof(header_t));
}

void TcpSession::start()
{
	startRead();
}

std::string TcpSession::getRemoteAddress()
{
	try{ return socket_.remote_endpoint().address().to_string(); }
	catch(...){ return std::string(""); }
}

void TcpSession::startRead()
{
	auto self(shared_from_this());

	boost::asio::async_read(socket_,
			boost::asio::buffer(header_buffer_),
			boost::bind(&TcpSession::handleReceiveHeader, this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));
}


void TcpSession::handleReceiveHeader(
		const boost::system::error_code& error,
		std::size_t bytes_transferred)
{
	if (!error || error == boost::asio::error::message_size)
	{
		header_t data_size = *(header_t*)(&header_buffer_[0]);

		data_buffer_.resize(data_size);

		boost::asio::async_read(socket_,
				boost::asio::buffer(data_buffer_),
				boost::bind(&TcpSession::handleReceiveData, this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));
	}
	else
	{
		std::cerr << "Error while receiving data: " << error.message() << std::endl;
	}
}


void TcpSession::handleReceiveData(
		const boost::system::error_code& error,
		std::size_t bytes_transferred)
{
	if (!error || error == boost::asio::error::message_size)
	{
		(*process_data_)(data_buffer_.data(), data_buffer_.size());

		uint64_t response = 1;
		boost::asio::write(socket_, boost::asio::buffer(&response, sizeof(uint64_t)));

		startRead();
	}
	else
	{
		std::cerr << "Error while receiving data: " << error.message() << std::endl;
	}
}


TcpServer::TcpServer(boost::asio::io_service& io_service, short port, TcpDataFunctor* data_functor):
			acceptor_(io_service, tcp::endpoint(tcp::v4(), port)),
			socket_(io_service)
{
	startAccept(data_functor);
}


boost::asio::io_service& TcpServer::get_io_service()
{
	return acceptor_.get_io_service();
}

void TcpServer::startAccept(TcpDataFunctor* data_functor)
{
	acceptor_.async_accept(socket_,
			boost::bind(&TcpServer::handleAccept, this,
					boost::asio::placeholders::error,
					data_functor));
}

void TcpServer::handleAccept(const boost::system::error_code& error, TcpDataFunctor* data_functor)
{
	if (!error)
	{
		std::shared_ptr<TcpSession> s = std::make_shared<TcpSession>(std::move(socket_), data_functor);
		s->start();
		sessions_.push_back(s);
		std::cout << "Connection accepted from " << s->getRemoteAddress() << std::endl;
	}
	else
	{
		std::cerr << error.message() << std::endl;
	}
	startAccept(data_functor);
}



