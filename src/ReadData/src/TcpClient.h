//
// Created by qkgautier on 2/13/17.
//

#ifndef SERVERAPP_TCPCLIENT_H
#define SERVERAPP_TCPCLIENT_H

//
// async_tcp_client.cpp
// ~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2010 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <iostream>



class TcpClient
{
public:

	typedef uint64_t response_t;

    TcpClient(boost::asio::io_service& io_service);

    // Called by the user of the client class to initiate the connection process.
    // The endpoint iterator will have been obtained using a tcp::resolver.
    void start(boost::asio::ip::tcp::resolver::iterator endpoint_iter);

    // This function terminates all the actors to shut down the connection. It
    // may be called by the user of the client class, or by the class itself in
    // response to graceful termination or an unrecoverable error.
    void stop();

    bool isStopped() const { return stopped_; }
    bool isConnected() const { return connected_; }

    void send_async(const char* data, size_t data_size);
    void send(const char* data, size_t data_size);

    response_t getLastResponse() const { return last_response_; }
    void resetResponse(){ last_response_ = 0; }

    static void runClient_task(
    		std::shared_ptr<TcpClient>& client,
    		std::string client_addr,
    		std::string client_port,
			boost::asio::io_service* io_service);

private:
    void start_connect(boost::asio::ip::tcp::resolver::iterator endpoint_iter);

    void handle_connect(const boost::system::error_code& ec,
                        boost::asio::ip::tcp::resolver::iterator endpoint_iter);

    void start_read();

    void handle_read(const boost::system::error_code& ec);

    void handle_write(const boost::system::error_code& ec);

    void check_deadline();

private:
    bool stopped_;
    bool connected_;
    boost::asio::ip::tcp::socket socket_;
    boost::array<response_t, 1>  input_buffer_;
    boost::asio::deadline_timer  deadline_;
    response_t last_response_;
};

#endif //SERVERAPP_TCPCLIENT_H
