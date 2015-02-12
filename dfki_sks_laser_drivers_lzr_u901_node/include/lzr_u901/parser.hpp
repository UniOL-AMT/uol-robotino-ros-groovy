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

#ifndef __LZR_U901_PARSER_HPP__
#define __LZR_U901_PARSER_HPP__

#include <boost/bind.hpp>
#include <boost/signal.hpp>

#include <timeutils/time.hpp>

#define LZR_U901_AS_HEX(x) "0x" << std::hex << (x) << std::dec
#define LZR_U901_CHAR_AS_HEX(x) LZR_U901_AS_HEX(int((x)) & 0xff)

#include "crc.hpp"
#include "scan.hpp"
#include "logging_defaults.hpp"

namespace lzr_u901 {

class parser
{
	typedef parser self;

	typedef enum
	{
		SYNC_1,
		SYNC_2,
		RANGE_DATA_1,
		RANGE_DATA_2,
		CRC_1,
		CRC_2
	} expected_byte_t;

	enum
	{
		SYNC_BYTE = 0xfe
	};

	enum
	{
		NUM_SYNC_BYTES = 2,
		NUM_CRC_BYTES = 2
	};

public:
	parser()
		: current_range_(0)
  	{
		LZR_U901_LOG_INFO("Parser starting up...");
		
		backtrack_buf_.reserve(scan::TOTAL_RANGES + NUM_CRC_BYTES);
		reset();
	}

	template <typename ScanHandler>
	void register_scan_handler(ScanHandler h)
	{
		deliver_scan_.connect(h);
	}

	void reset()
	{
		LZR_U901_LOG_TRACE("parser reset");

		backtrack_buf_.clear();

		crc_.reset();
		scan_.clear();
		
		expected_ = SYNC_1;
	}

	bool parse(const uint8_t &b)
	{
		return handle_char(b);
	}

private:

	bool check_checksum()
	{
		uint16_t current_crc = crc_.current();

		bool correct = current_crc == checksum_;

		if (correct)
		{
			LZR_U901_LOG_TRACE("checksum matches: " << LZR_U901_AS_HEX(current_crc));
		}
		else
		{
			LZR_U901_LOG_TRACE("incorrect checksum: calculated=" <<  LZR_U901_AS_HEX(current_crc) << " control=" << LZR_U901_AS_HEX(checksum_));
		}
		
		return correct;
	}

	bool handle_char(const uint8_t &c)
	{
		bool scan_complete = false;
		
		switch(expected_)
		{
		case SYNC_1:
			if (c == SYNC_BYTE)
			{
				LZR_U901_LOG_TRACE("SYNC_1 found");
				expected_ = SYNC_2;
			}
			else
			{
				//LZR_U901_LOG_WARN("SYNC_1 not found");
			}
			break;
		case SYNC_2:
			if (c == SYNC_BYTE)
			{
				timeutils::get_current_time(&scan_.time_received);

				LZR_U901_LOG_TRACE("SYNC_2 found");
				backtrack_buf_.push_back(c);
				expected_ = RANGE_DATA_1;
			}
			else
			{
				LZR_U901_LOG_TRACE("SYNC_2 not found");
				expected_ = SYNC_1;
			}
			break;
		case RANGE_DATA_1:
			backtrack_buf_.push_back(c);
			crc_.update(c);
			current_range_ = (((uint16_t) c) & 0xff) << 8;
			expected_ = RANGE_DATA_2;
			break;
		case RANGE_DATA_2:
			backtrack_buf_.push_back(c);
			crc_.update(c);
			current_range_ |= c;
			scan_.push_back(current_range_);
			if (scan_.size() == scan::TOTAL_RANGES)
			{
				LZR_U901_LOG_TRACE("TOTAL_RANGES reached. now expecting CRC_1");
				expected_ = CRC_1;
			}
			else
			{
				expected_ = RANGE_DATA_1;
			}
			break;
		case CRC_1:
			LZR_U901_LOG_TRACE("Storing CRC_1");
			backtrack_buf_.push_back(c);
			checksum_ = ((uint16_t) c) << 8;
			expected_ = CRC_2;
			break;
		case CRC_2:
			LZR_U901_LOG_TRACE("Storing CRC_2 and checking checksum");
			backtrack_buf_.push_back(c);
			checksum_ |= c;
			if (check_checksum())
			{
				LZR_U901_LOG_DEBUG("checksum matches. delivering scan");
				deliver_scan_(scan_);
				reset();
				scan_complete = true;
			}
			else
			{
				LZR_U901_LOG_WARN("checksum mismatch. re-sync");
				
				std::vector<uint8_t> tmp = backtrack_buf_;

				reset();
				LZR_U901_LOG_TRACE("processing backtrack_buf");
				for(std::vector<uint8_t>::const_iterator i = tmp.begin();
					i != tmp.end(); ++i)
				{
					handle_char(*i);
				}
				LZR_U901_LOG_TRACE("done processing backtrack_buf");
			}
			break;
		}

		return scan_complete;
	}

	std::vector<uint8_t> backtrack_buf_;
	
	expected_byte_t expected_;

	scan scan_;
	scan::range current_range_;

	crc crc_;
	uint16_t checksum_;

	boost::signal<void (const scan&)> deliver_scan_;
};

} // namespace lzr_u901

#endif // __LZR_U901_PARSER_HPP__
