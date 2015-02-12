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

#ifndef __LZR_U901_BASE_NODE_HPP__
#define __LZR_U901_BASE_NODE_HPP__

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sensor_msgs/LaserScan.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <lzr_u901/range_to_point_converter.hpp>

class lzr_u901_base_node
{

	typedef lzr_u901_base_node self;
	typedef pcl::PointCloud<pcl::PointWithRange> PointCloud;
	
public:
	lzr_u901_base_node(ros::NodeHandle &nh,
					   const std::string &frame_id)
		: frame_id_(frame_id),
		  scan_pub_(nh.advertise<sensor_msgs::LaserScan>("scan", 1)),
		  point_cloud_pub_(nh.advertise<PointCloud>(nh.resolveName("point_cloud"), 1))
	{}
	
protected:
	void publish_plane_as_laser_scan(const lzr_u901::scan &scan, const lzr_u901::scan::plane_t p)
	{
		using namespace lzr_u901;
		
		sensor_msgs::LaserScan scan_msg;
		scan_msg.header.stamp.fromNSec(1000*scan.time_received);
		scan_msg.header.frame_id = frame_id_;

		int num_ranges = scan::NUM_RANGES_PER_PLANE;
		
		scan_msg.angle_min = scan::angle_min();
		scan_msg.angle_max = scan::angle_max();
		scan_msg.angle_increment = scan::angle_increment();
		scan_msg.scan_time = 1/15.;
		scan_msg.time_increment = scan_msg.scan_time / (double)(num_ranges-1);
		scan_msg.range_min = 0;
		scan_msg.range_max = 15.;
		
		scan_msg.ranges.reserve(num_ranges);
		foreach (const scan::range &r, scan.plane(p))
		{
			scan_msg.ranges.push_back(scan::range_to_meters(r));
		}
		scan_pub_.publish(scan_msg);
	}

private:
	void append_to_point_cloud(pcl::PointCloud<pcl::PointWithRange> &cloud,
							   const float &x, const float &y, const float &z,
							   const float &r) const
	{
		pcl::PointWithRange p;
		p.x = x; p.y = y; p.z = z; p.range = r;

		cloud.push_back(p);
	}

protected:
	void publish_point_cloud(const lzr_u901::scan &scan)
	{
		using namespace lzr_u901;
		pcl::PointCloud<pcl::PointWithRange> cloud;
		cloud.header.frame_id = frame_id_;

		// populate cloud
		for (unsigned p = scan::PLANE_0; p < scan::NUM_PLANES; ++p)
		{
			point_converter_.convert(scan::plane_t(p), scan.plane(scan::plane_t(p)), boost::bind(&self::append_to_point_cloud, this, boost::ref(cloud), _1, _2, _3, _4));
		}

		point_cloud_pub_.publish(cloud);
	}
	
	void handle_scan(const lzr_u901::scan &scan)
	{
		using namespace lzr_u901;

		publish_plane_as_laser_scan(scan, scan::PLANE_0);
		publish_point_cloud(scan);
	}
	
private:
	std::string frame_id_;
	ros::Publisher scan_pub_;

	lzr_u901::range_to_point_converter point_converter_;
	ros::Publisher point_cloud_pub_;
};

#endif // __LZR_U901_BASE_NODE_HPP__
