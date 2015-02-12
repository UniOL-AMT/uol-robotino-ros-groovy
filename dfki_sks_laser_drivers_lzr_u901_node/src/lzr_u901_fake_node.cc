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

#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/range.hpp>

#include <ros/ros.h>

#include "lzr_u901_ros_logging.hpp"
#include <lzr_u901/parser.hpp>

#include "lzr_u901_base_node.hpp"

using namespace boost::interprocess;

class lzr_u901_fake_node : public lzr_u901_base_node
{
	typedef lzr_u901_fake_node self;
	typedef const uint8_t * log_iterator;
	
public:
	lzr_u901_fake_node(ros::NodeHandle &nh,
			  const std::string &filename,
			  const std::string &frame_id)
		: lzr_u901_base_node(nh, frame_id),
		  m_file_(filename.c_str(), read_only),
		  m_region_(m_file_, read_only),
		  log_iter_(static_cast<log_iterator>(m_region_.get_address())),
		  log_(log_iter_, log_iter_ + m_region_.get_size())
	{
		parser_.register_scan_handler(boost::bind(&self::handle_scan, this, _1));
		
		ROS_INFO_STREAM("Attempting to parse " << log_.size() << " bytes.");
	}

	bool parse_one()
	{
		bool scan_complete = false;
		
		for (; !scan_complete && more_data(); ++log_iter_)
		{
			scan_complete = parser_.parse(*log_iter_);
		}
		
		return more_data();
	}

	bool more_data() const
	{
		return log_iter_ != log_.end();
	}

private:
	lzr_u901::parser parser_;
	
	file_mapping m_file_;
	mapped_region m_region_;
	
	log_iterator log_iter_;	
	boost::iterator_range<log_iterator> log_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lzr_u901_fake_node");

	ROSCONSOLE_AUTOINIT;

	log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

	// Set the logger for this package to output all statements
	my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

	std::string filename, frame_id;

	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	nh_ns.param<std::string>("dump", filename, "lzr_u901.dump");
	nh_ns.param<std::string>("frame_id", frame_id, "laser");

	ROS_INFO_STREAM("reading dump from file: " << filename);

	try {
		lzr_u901_fake_node node(nh, filename, frame_id);

		ros::Rate loop_rate(15);
		
		while (ros::ok())
		{
			bool more = node.parse_one();
			ros::spinOnce();

			if (!more)
				break;

			loop_rate.sleep();
		}
		
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
