/*
 * Copyright (c) 2015, Thomas Keh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#pragma once

#include <string>
#include <map>
#include <regex>
 
#include <boost/property_tree/ptree.hpp> // property_tree

#include "entry.h"
#include "types.h"

namespace kaco {

/// This class allows reading EDS files (like standardized in CiA 306)
/// and inserting all contained entries into a dictionary of type
/// std::map<std::string, Entry>.
/// It makes use of Boost's property_tree class.
class EDSReader {

public:

	/// Constructor.
	/// \param target The dictionary, into which entries should be inserted.
	EDSReader(std::map<std::string, Entry>& target);

	/// Loads an EDS file from file system.
	/// \returns true if successful
	bool load_file(std::string filename);

	/// Import entries from the EDS file into the given dictionary
	/// \returns true if successful
	bool import_entries();

private:

	/// Enable debug logging.
	static const bool debug = false;

	/// reference to the dictionary
	std::map<std::string, Entry>& m_map;
	
	/// This property tree represents the EDS file imported in load_file().
	/// EDS files have the same syntax like Windows INI files.
	boost::property_tree::ptree m_ini;

	/// Parse an index section (e.g. [1000])
	bool parse_index(const std::string& section, uint16_t index);
	
	/// Parse a section which represents a variable. Can be an index section like [1000] or a subindex section like [1018sub0].
	bool parse_var(const std::string& section, uint16_t index, uint8_t subindex, const std::string& name_prefix = "");
	
	/// Parse an index section which has ObjectType array or record.
	bool parse_array_or_record(const std::string& section, uint16_t index);

	/// Parses a regex error. This is just for debugging purposes.
	std::string parse_regex_error(const std::regex_constants::error_type& etype) const;


};

} // end namespace kaco