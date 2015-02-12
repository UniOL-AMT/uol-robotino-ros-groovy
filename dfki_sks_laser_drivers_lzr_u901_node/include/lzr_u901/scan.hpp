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

#ifndef __LZR_U901_SCAN_HPP__
#define __LZR_U901_SCAN_HPP__

#include <stdint.h>

#include <vector>

#include <boost/range.hpp>

namespace lzr_u901 {

struct scan
{
	typedef uint16_t range;
	
	typedef std::vector<range> ranges_vector;
	typedef ranges_vector::const_iterator ranges_iterator;
	typedef ranges_vector::size_type size_type;

	typedef boost::iterator_range<ranges_vector::const_iterator> ranges_view;

	typedef float rad;

	enum
	{
		NUM_RANGES_PER_PLANE = 274,
		NUM_PLANES = 4,
		TOTAL_RANGES = NUM_PLANES * NUM_RANGES_PER_PLANE
	};

	typedef enum
	{
		PLANE_0 = 0,
		PLANE_1,
		PLANE_2,
		PLANE_3
	} plane_t;
	
	scan()
	{
		ranges_.reserve(TOTAL_RANGES);
	}

	void clear()
	{
		ranges_.clear();
	}

	void push_back(const range &r)
	{
		ranges_.push_back(r);
	}

	size_type size() const
	{
		return ranges_.size();
	}

	ranges_view plane(plane_t i) const
	{
		static const size_type lut[] = {2, 0, 3, 1};

		return plane_slot(lut[i]);
	}

	static float range_to_meters(const range &r)
	{
		return 0.001 * r;
	}

	static rad plane_angle_increment()
	{
		return 2. * M_PI/180.;
	}
	
	static rad plane_angle(plane_t i)
	{

		return i * plane_angle_increment();
	}

	static rad angle_min()
	{
		return -48. * M_PI/180.;
	}

	static rad angle_max()
	{
		return 48. * M_PI/180.;
	}

	static rad angle_increment()
	{
		return (angle_max() - angle_min()) / (float)(NUM_RANGES_PER_PLANE - 1);
	}

	static float max_range_meters()
	{
		return 9.9f;
	}

	uint64_t time_received;
		
protected:

	ranges_view plane_slot(size_type i) const
	{
		return ranges_view(ranges_.begin() + i * NUM_RANGES_PER_PLANE,
						   ranges_.begin() + (i+1) * NUM_RANGES_PER_PLANE);
	}
	
	ranges_vector ranges_;
};
	
} // namespace lzr_u901

#endif // __LZR_U901_SCAN_HPP__
