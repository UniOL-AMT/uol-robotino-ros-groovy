/*
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

#include <sstream>

#include <ros/ros.h>
#include <ros/poll_manager.h>
#include <ros/poll_set.h>

#include "lzr_u901_ros_logging.hpp"
#include <lzr_u901/serial_client.hpp>

#include "lzr_u901_base_node.hpp"

class lzr_u901_client_node : public lzr_u901::serial_client, public lzr_u901_base_node
{
	typedef lzr_u901_client_node self;
	
public:
	lzr_u901_client_node(ros::NodeHandle &nh,
			  boost::asio::io_service& io_service,
			  const std::string &device,
			  const std::string &frame_id)
		: lzr_u901::serial_client(io_service, device, boost::bind(&self::handle_asio_error, this, _1)),
		  lzr_u901_base_node(nh, frame_id)
	{
		int fd = serial_port_.native();
		ros::PollSet &poll_set = ros::PollManager::instance()->getPollSet();
		poll_set.addSocket(fd, boost::bind(&self::handle_socket_io, this));
		poll_set.addEvents(fd, POLLIN|POLLPRI|POLLERR|POLLHUP);
		
		register_scan_handler(boost::bind(&self::handle_scan, this, _1));
	}

private:
	void handle_socket_io()
	{
		boost::system::error_code ec;
		io_service_.poll(ec);
		if (ec)
		{
			ROS_FATAL_STREAM("poll error: " << ec.message());
			ros::shutdown();
		}
	}

	void handle_asio_error(const boost::system::error_code &error)
	{
		ros::shutdown();
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lzr_u901_node");

	std::string device;
	std::string frame_id;

	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	nh_ns.param("device", device, std::string("/dev/ttyUSB0"));
	nh_ns.param<std::string>("frame_id", frame_id, "laser");

	try {
		boost::asio::io_service io_service;
		lzr_u901_client_node node(nh, io_service, device, frame_id);
	
		ros::spin();
	
		return 0;
	}
	catch (std::exception &e)
	{
		ROS_FATAL_STREAM("exception: " << e.what());
	}
	catch (const char *e)
	{
		ROS_FATAL_STREAM("exception: " << e);
	}
	
	return 1;
}
