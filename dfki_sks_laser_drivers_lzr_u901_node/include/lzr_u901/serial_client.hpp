/*
 *  Copyright (c) 2009, Rene Wagner
 *  Copyright (c) 2010, DFKI GmbH
 *  All rights reserved.
 *
 *  Author: Rene Wagner <rene.wagner@dfki.de>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the DFKI GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __LZR_U901_SERIAL_CLIENT_HPP__
#define __LZR_U901_SERIAL_CLIENT_HPP__

#include <string>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/signal.hpp>

#include <timeutils/time.hpp>

#include "parser.hpp"
#include "scan.hpp"

#include "logging_defaults.hpp"

namespace lzr_u901 {

class serial_client
{
	typedef serial_client self;

	enum
	{
		BAUD_RATE = 460800
	};

public:
	template<typename ErrorHandler>
	serial_client(boost::asio::io_service& io_service, const std::string &device,
				  ErrorHandler h)
		: io_service_(io_service),
		  serial_port_(io_service, device),
		  error_handler_(h),
		  connected_(false)
  	{
		if (not serial_port_.is_open()) {
			throw "Could not open serial port";
		}

		// configure serial port
		boost::asio::serial_port_base::baud_rate baud_rate(BAUD_RATE);
		serial_port_.set_option(baud_rate);

		// defaults are OK for all other options (8-N-1)

		LZR_U901_LOG_INFO("connection established");
		connected_ = true;

		start();
	}

	template <typename ScanHandler>
	void register_scan_handler(ScanHandler h)
	{
		parser_.register_scan_handler<ScanHandler>(h);
	}

	bool connected() const
	{
		return connected_;
	}

	void close()
	{
		io_service_.post(boost::bind(&self::do_close, this));
	}
	
private:
	void start()
	{
		parser_.reset();
		serial_port_.async_read_some(boost::asio::buffer(raw_buf_),
								boost::bind(&self::handle_inbound,
											this,
											boost::asio::placeholders::error,
											boost::asio::placeholders::bytes_transferred));

	}
	
	void handle_inbound(const boost::system::error_code &error, std::size_t bytes_read) {
		if (!error) {
			LZR_U901_LOG_TRACE("got data " << bytes_read);
			//printHex((const uint8_t *) raw_buf_.data(), bytes_read);
			for (const uint8_t *i = raw_buf_.data(); bytes_read; ++i, --bytes_read) {
				parser_.parse(*i);
			}
			serial_port_.async_read_some(boost::asio::buffer(raw_buf_),
									boost::bind(&self::handle_inbound,
												this,
												boost::asio::placeholders::error,
												boost::asio::placeholders::bytes_transferred));
		} else {
			close(error);
		}
	}
	
	void close(const boost::system::error_code &error)
	{
		LZR_U901_LOG_FATAL("IO error: " << error.message());
		do_close();
		error_handler_(error);
	}
	
	void do_close()
	{
		serial_port_.close();
		connected_ = false;
	}
	
protected:
	boost::asio::io_service& io_service_;
	boost::asio::serial_port serial_port_;

private:
	boost::function<void (const boost::system::error_code &)> error_handler_;

	boost::array<uint8_t, 512> raw_buf_;

	parser parser_;

	bool connected_;

};

} // namespace lzr_u901

#endif // __LZR_U901_SERIAL_CLIENT_HPP__
