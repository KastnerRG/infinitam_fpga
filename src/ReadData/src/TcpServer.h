/*
 * TcpServer.h
 *
 *  Created on: Feb 14, 2017
 *      Author: qkgautier
 */

#ifndef SRC_TCPSERVER_H_
#define SRC_TCPSERVER_H_


#include <boost/asio.hpp>

class TcpDataFunctor
{
public:
	TcpDataFunctor(){}
	virtual ~TcpDataFunctor(){}
	virtual void operator()(const char*, size_t) = 0;
};


class TcpSession: public std::enable_shared_from_this<TcpSession>
{
public:

	typedef uint64_t header_t;

	TcpSession(boost::asio::ip::tcp::socket socket, TcpDataFunctor* data_functor);

	void start();

	std::string getRemoteAddress();

private:
	void startRead();

	void handleReceiveHeader(
			const boost::system::error_code& error,
			std::size_t bytes_transferred);

	void handleReceiveData(
			const boost::system::error_code& error,
			std::size_t bytes_transferred);


	boost::asio::ip::tcp::socket socket_;

	std::vector<char> header_buffer_;
	std::vector<char> data_buffer_;

	TcpDataFunctor* process_data_;
};


class TcpServer
{
public:
	TcpServer(boost::asio::io_service& io_service, short port, TcpDataFunctor* data_functor);

	const std::vector< std::shared_ptr<TcpSession> >& getSessions(){ return sessions_; }

	boost::asio::io_service& get_io_service();

private:
	void startAccept(TcpDataFunctor* data_functor);

	void handleAccept(const boost::system::error_code& error, TcpDataFunctor* data_functor);

	boost::asio::ip::tcp::acceptor acceptor_;
	boost::asio::ip::tcp::socket   socket_;

	std::vector< std::shared_ptr<TcpSession> > sessions_;
};








#endif /* SRC_TCPSERVER_H_ */
