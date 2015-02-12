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

#ifndef __LZR_U901_RANGE_TO_POINT_CONVERTER_HPP__
#define __LZR_U901_RANGE_TO_POINT_CONVERTER_HPP__

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "scan.hpp"

namespace lzr_u901 {

class range_to_point_converter
{
	typedef Eigen::Vector3f point;
	typedef std::vector<point, Eigen::aligned_allocator<point> > directions;

public:
	range_to_point_converter()
	{
		populate_directions_table();
	}

	template<typename RangesContainer, typename PointHandler>
	void convert(scan::plane_t plane, const RangesContainer &ranges, PointHandler h) const
	{
		typename RangesContainer::const_iterator r = ranges.begin();
		directions::const_iterator d = direction_table[plane].begin();

		for (; r != ranges.end() && d != direction_table[plane].end(); ++r, ++d)
		{
			float r_meters = scan::range_to_meters(*r);
			point p = *d * r_meters;
			h(p.x(), p.y(), p.z(), r_meters);
		}
	}

private:
	static scan::rad plane_angle_increment()
	{
		return -2. * M_PI/180.;
	}
	
	void populate_directions_table()
	{
		directions not_rotated;
		not_rotated.reserve(scan::NUM_RANGES_PER_PLANE);
		float t = scan::angle_min();
		for (unsigned k = 0; k < scan::NUM_RANGES_PER_PLANE; ++k, t += scan::angle_increment())
		{
			not_rotated.push_back(point(cos(t), sin(t), 0));
		}
		
		for (unsigned p = scan::PLANE_0; p < scan::NUM_PLANES; ++p)
		{
			directions &tab = direction_table[p];
			tab.reserve(scan::NUM_RANGES_PER_PLANE);

			Eigen::AngleAxis<float> rot(p * plane_angle_increment(), point(0., 1., 0.));
			
			for (directions::const_iterator i = not_rotated.begin();
				 i != not_rotated.end(); ++i)
				tab.push_back(rot * *i);
		}
	}
	
	directions direction_table[scan::NUM_PLANES];
};
	
} // namespace lzr_u901

#endif // __LZR_U901_RANGE_TO_POINT_CONVERTER_HPP__
