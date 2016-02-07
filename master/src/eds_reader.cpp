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

#include "logger.h"
#include "eds_reader.h"
#include "entry.h"
#include "utils.h"

#include <regex>
#include <string>
#include <cassert>

#include <boost/property_tree/ptree.hpp> // property_tree
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string/predicate.hpp> // string::starts_with()

namespace kaco {

EDSReader::EDSReader(std::map<std::string, Entry>& target)
	: m_map(target)
	{ }

bool EDSReader::load_file(std::string filename) {

	try {
		boost::property_tree::ini_parser::read_ini(filename, m_ini);
    	return true;
	} catch (const std::exception& e) {
		ERROR("[EDSReader::load_file] Could not open file: "<<e.what());
    	return false;
	}

}

bool EDSReader::import_entries() {

	bool success = true;

    for (const auto& section_node : m_ini) {
		const std::string& section_name = section_node.first;
		const boost::property_tree::ptree& section = section_node.second;

    	try {
    		
    		std::smatch matches;

	    	if (std::regex_match(section_name, std::regex("[[:xdigit:]]{1,4}"))) {
	    		
	    		uint16_t index = (uint16_t) Utils::hexstr_to_uint(section_name);
	    		DEBUG_LOG_EXHAUSTIVE("Section "<<section_name<<" corresponds to index "<<index<<".");
	    		success = parse_index(section_name, index) && success; // mind order!

	    	}
	    	/*
	    	else if (std::regex_match(section_name, matches, std::regex("([[:xdigit:]]{1,4})sub([[:xdigit:]]{1,2})"))) {

	    		// Ignoring subindex entries here!

	    	} else {

	    		// Ignoring metadata entries here!
	    		DEBUG_LOG_EXHAUSTIVE("Section "<<section_name<<" contains meta data. Doing nothing.");

	    	}
	    	*/

	    } catch (std::regex_error& e) {
	    	ERROR("[EDSReader::import_entries] " << parse_regex_error(e.code()));
	    	success = false;
		}

    }

	return success;

}

bool EDSReader::parse_index(const std::string& section, uint16_t index) {

	std::string str_object_type = m_ini.get(section+".ObjectType", "");
	
	uint8_t object_code = (uint8_t) ObjectType::VAR;
	if (str_object_type.empty()) {
		DEBUG_LOG("Field ObjectType missing. Assuming ObjectType::VAR (according to DS 306 V1.3 page 16).");
	} else {
		object_code = (uint8_t) Utils::hexstr_to_uint(str_object_type);
	}

	
	if (object_code == (uint8_t) ObjectType::VAR) {
		return parse_var(section, index, 0);
	} else if ((object_code == (uint8_t) ObjectType::RECORD) || (object_code == (uint8_t) ObjectType::ARRAY)) {
		return parse_array_or_record(section, index);
	}

	DEBUG_LOG("This is not a variable and no array. Ignoring.")
	return true;

}

bool EDSReader::parse_var(const std::string& section, uint16_t index, uint8_t subindex, const std::string& name_prefix) {

	std::string var_name = Utils::escape(m_ini.get(section+".ParameterName", ""));

	if (var_name.empty()) {
		ERROR("[EDSReader::parse_var] Field ParameterName missing");
		return false;
	}

	if (!name_prefix.empty()) {
		var_name = name_prefix + "/" + var_name;
	}

	DEBUG_LOG("[EDSReader::parse_var] Parsing variable "<<section<<": "<<var_name);

	std::string str_sub_number = m_ini.get(section+".SubNumber", "");
	std::string str_object_type = m_ini.get(section+".ObjectType", "");
	std::string str_data_type = m_ini.get(section+".DataType", "");
	std::string str_low_limit = m_ini.get(section+".LowLimit", "");
	std::string str_high_limit = m_ini.get(section+".HighLimit", "");
	std::string str_access_type = m_ini.get(section+".AccessType", "");
	std::string str_default_value = m_ini.get(section+".DefaultValue", "");
	std::string str_pdo_mapping = m_ini.get(section+".PDOMapping", "");
	std::string str_obj_flags = m_ini.get(section+".ObjFlags", "");

	Entry entry;
	entry.name = var_name;
	entry.type = Utils::type_code_to_type((uint16_t) Utils::hexstr_to_uint(str_data_type));
	entry.index = index;
	entry.subindex = subindex;

	if (entry.type == Type::invalid) {
		ERROR("[EDSReader::parse_var] Invalid data type: "<<str_data_type<<" / 0x"<<std::hex<<(uint16_t) Utils::hexstr_to_uint(str_data_type));
		DUMP(Utils::type_to_string(entry.type));
		DUMP(Utils::type_to_string(Utils::type_code_to_type((uint16_t) Utils::hexstr_to_uint(str_data_type))));
		return false;
	}

	entry.access_type = Utils::string_to_access_type(str_access_type);

	// --- insert entry --- //

	while (m_map.count(var_name)>0) {
		
		WARN("[EDSReader::parse_var] Entry "<<var_name<<" already exists. Adding or increasing counter.");
		
		try {
			std::smatch matches;
			if (std::regex_match(var_name, matches, std::regex("(.+)_([[:xdigit:]]{1,3})"))) {
		    	assert(matches.size()>2);
		    	uint8_t count = Utils::decstr_to_uint(matches[2]);
		    	++count;
		    	var_name = std::string(matches[1])+"_"+std::to_string(count);
			} else {
				var_name = var_name+"_1";
			}
		} catch (std::regex_error& e) {
		    WARN("[EDSReader::parse_var] "<<parse_regex_error(e.code()));
		    return false;
		}

		DEBUG_LOG("[EDSReader::parse_var] New entry name: "<<var_name);
		entry.name = var_name;

	}

	DEBUG_LOG("[EDSReader::parse_var] Inserting entry "<<var_name<<".");
	m_map[var_name] = entry;

	return true;

}

bool EDSReader::parse_array_or_record(const std::string& section, uint16_t index) {
	
	std::string array_name = Utils::escape(m_ini.get(section+".ParameterName", ""));

	if (array_name.empty()) {
		ERROR("[EDSReader::parse_array_or_record] Field ParameterName missing");
		return false;
	}

	DEBUG_LOG_EXHAUSTIVE("[EDSReader::parse_array_or_record] Parsing array/record "<<section<<": "<<array_name);

	for (const auto& section_node : m_ini) {
		const std::string& section_name = section_node.first;
		const boost::property_tree::ptree& parameters = section_node.second;

		if (boost::starts_with(section_name, section)) {
	
			DEBUG_LOG_EXHAUSTIVE("[EDSReader::parse_array_or_record] Found record/array entry: "<<section_name);
    	
    		try {
    			
    			std::smatch matches;

				if (std::regex_match(section_name, matches, std::regex("([[:xdigit:]]{1,4})sub([[:xdigit:]]{1,2})"))) {

		    		assert(matches.size()>2);
		    		assert(Utils::hexstr_to_uint(matches[1])==index);
		    		uint8_t subindex = Utils::hexstr_to_uint(matches[2]);
		    		parse_var(section_name, index, subindex, array_name);

		    	} else if (section_name == section) {
		    		// ignore own entry
		    		continue;
		    	} else {
		    		ERROR("[EDSReader::parse_array_or_record] Malformed array entry: "<<section_name);
		    		return false;
		    	}

		    } catch (std::regex_error& e) {
		    	ERROR("[EDSReader::parse_array_or_record] "<<parse_regex_error(e.code()));
		    	return false;
			}


		}

    }

	return true;

}

std::string EDSReader::parse_regex_error(const std::regex_constants::error_type& etype) const {
	switch (etype) {
	    case std::regex_constants::error_collate:
	        return "error_collate: invalid collating element request";
	    case std::regex_constants::error_ctype:
	        return "error_ctype: invalid character class";
	    case std::regex_constants::error_escape:
	        return "error_escape: invalid escape character or trailing escape";
	    case std::regex_constants::error_backref:
	        return "error_backref: invalid back reference";
	    case std::regex_constants::error_brack:
	        return "error_brack: mismatched bracket([ or ])";
	    case std::regex_constants::error_paren:
	        return "error_paren: mismatched parentheses(( or ))";
	    case std::regex_constants::error_brace:
	        return "error_brace: mismatched brace({ or })";
	    case std::regex_constants::error_badbrace:
	        return "error_badbrace: invalid range inside a { }";
	    case std::regex_constants::error_range:
	        return "erro_range: invalid character range(e.g., [z-a])";
	    case std::regex_constants::error_space:
	        return "error_space: insufficient memory to handle this regular expression";
	    case std::regex_constants::error_badrepeat:
	        return "error_badrepeat: a repetition character (*, ?, +, or {) was not preceded by a valid regular expression";
	    case std::regex_constants::error_complexity:
	        return "error_complexity: the requested match is too complex";
	    case std::regex_constants::error_stack:
	        return "error_stack: insufficient memory to evaluate a match";
	    default:
	        return "";
    }
}

} // end namespace kaco