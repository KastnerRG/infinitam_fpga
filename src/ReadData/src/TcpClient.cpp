//
// Created by qkgautier on 2/13/17.
//

#include "TcpClient.h"

#include <boost/bind.hpp>

using boost::asio::deadline_timer;
using boost::asio::ip::tcp;


#ifdef __ANDROID__
#include "logging.h"
#else
#define  LOGE(...)  { printf(__VA_ARGS__); printf("\n"); }
#define  LOGW(...)  { printf(__VA_ARGS__); printf("\n"); }
#define  LOGD(...)  { printf(__VA_ARGS__); printf("\n"); }
#define  LOGI(...)  { printf(__VA_ARGS__); printf("\n"); }
#endif


TcpClient::TcpClient(boost::asio::io_service& io_service):
        stopped_(false),
        connected_(false),
        socket_(io_service),
        deadline_(io_service)
{
	resetResponse();
}

void TcpClient::start(tcp::resolver::iterator endpoint_iter)
{
    // Start the connect actor.
    start_connect(endpoint_iter);

    // Start the deadline actor. You will note that we're not setting any
    // particular deadline here. Instead, the connect and input actors will
    // update the deadline prior to each asynchronous operation.
    deadline_.async_wait(boost::bind(&TcpClient::check_deadline, this));
}

void TcpClient::stop()
{
    stopped_   = true;
    connected_ = false;
    socket_.close();
    deadline_.cancel();
}

void TcpClient::send_async(const char* data, size_t data_size)
{
    if(stopped_){ return; }

    boost::asio::async_write(socket_, boost::asio::buffer(data, data_size),
                             boost::bind(&TcpClient::handle_write, this, _1));
}

void TcpClient::send(const char* data, size_t data_size)
{
    if(stopped_){ return; }

    boost::asio::write(socket_, boost::asio::buffer(data, data_size));
}

void TcpClient::runClient_task(
		std::shared_ptr<TcpClient>& client,
		std::string client_addr,
		std::string client_port,
		boost::asio::io_service* io_service_ptr)
{
	boost::asio::io_service io_service;

	if(!io_service_ptr){ io_service_ptr = &io_service; }

	boost::asio::ip::tcp::resolver r(*io_service_ptr);

	client.reset(new TcpClient(*io_service_ptr));

	client->start(r.resolve(boost::asio::ip::tcp::resolver::query(client_addr, client_port)));

	io_service_ptr->run();
}


void TcpClient::start_connect(tcp::resolver::iterator endpoint_iter)
{
    if (endpoint_iter != tcp::resolver::iterator())
    {
        // Set a deadline for the connect operation.
        deadline_.expires_from_now(boost::posix_time::seconds(60));

        // Start the asynchronous connect operation.
        socket_.async_connect(endpoint_iter->endpoint(),
                              boost::bind(&TcpClient::handle_connect,
                                          this, _1, endpoint_iter));
    }
    else
    {
        // There are no more endpoints to try. Shut down the client.
        stop();
    }
}

void TcpClient::handle_connect(const boost::system::error_code& ec,
                               tcp::resolver::iterator endpoint_iter)
{
    if (stopped_)
        return;

    // The async_connect() function automatically opens the socket at the start
    // of the asynchronous operation. If the socket is closed at this time then
    // the timeout handler must have run first.
    if (!socket_.is_open())
    {
        LOGE("Connect timed out\n");

        // Try the next available endpoint.
        start_connect(++endpoint_iter);
    }

        // Check if the connect operation failed before the deadline expired.
    else if (ec)
    {
        LOGE("Connect error: %s", ec.message().c_str());

        // We need to close the socket used in the previous connection attempt
        // before starting a new one.
        socket_.close();

        // Try the next available endpoint.
        start_connect(++endpoint_iter);
    }

        // Otherwise we have successfully established a connection.
    else
    {
        connected_ = true;

        // for debug
        std::stringstream ss;
        ss << endpoint_iter->endpoint();
        std::string ss_str = ss.str();
        LOGI("Connected to %s", ss_str.c_str());

        // Start the input actor.
        start_read();
    }
}

void TcpClient::start_read()
{
    // Start an asynchronous operation to read data.
    boost::asio::async_read(socket_,
    		boost::asio::buffer(input_buffer_),
			boost::bind(&TcpClient::handle_read, this, _1));
}

void TcpClient::handle_read(const boost::system::error_code& ec)
{
    if (stopped_)
        return;

    if (!ec)
    {
    	last_response_ = input_buffer_[0];

        start_read();
    }
    else
    {
        LOGE("Error on receive: %s", ec.message().c_str());
        stop();
    }
}

void TcpClient::handle_write(const boost::system::error_code& ec)
{
    if (stopped_)
        return;

    if (!ec)
    {
        // Do something
    }
    else
    {
        LOGE("Error on write: %s", ec.message().c_str());

        stop();
    }
}

void TcpClient::check_deadline()
{
    if (stopped_)
        return;

    // Check whether the deadline has passed. We compare the deadline against
    // the current time since a new asynchronous operation may have moved the
    // deadline before this actor had a chance to run.
    if (deadline_.expires_at() <= deadline_timer::traits_type::now())
    {
        // The deadline has passed. The socket is closed so that any outstanding
        // asynchronous operations are cancelled.
        socket_.close();

        // There is no longer an active deadline. The expiry is set to positive
        // infinity so that the actor takes no action until a new deadline is set.
        deadline_.expires_at(boost::posix_time::pos_infin);
    }

    // Put the actor back to sleep.
    deadline_.async_wait(boost::bind(&TcpClient::check_deadline, this));
}




