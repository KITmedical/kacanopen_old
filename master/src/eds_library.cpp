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

#include "eds_library.h"
#include "eds_reader.h"
#include "logger.h"

#include <vector>
#include <string>
#include <boost/filesystem.hpp>

// This is set by CMake...
//#define SHARE_SOURCE_PATH ...
//#define SHARE_INSTALLED_PATH ...

namespace kaco {
	
	namespace fs = boost::filesystem;

	EDSLibrary::EDSLibrary(std::map<std::string, Entry>& target)
		: m_map(target), m_ready(false)
		{ }

	bool EDSLibrary::lookup_library(std::string path) {

		m_ready = false;

		std::vector<std::string> paths;
		if (!path.empty()) {
			paths.push_back(path+"/eds_library.txt");
		}

		paths.push_back(SHARE_SOURCE_PATH "/eds_library/eds_library.txt");
		paths.push_back(SHARE_INSTALLED_PATH "/eds_library/eds_library.txt");
		paths.push_back("/usr/local/share/kacanopen/eds_library/eds_library.txt");
		paths.push_back("/usr/share/kacanopen/eds_library/eds_library.txt");
		// TODO: typical windows / mac osx paths?

		bool success = false;
		for (const std::string& path : paths) {
			if (fs::exists(path)) {
				m_library_path = fs::path(path).parent_path().string();
				DEBUG_LOG("[EDSLibrary::lookup_library] Found EDS library in "<<m_library_path);
				success = true;
				break;
			}
		}

		if (!success) {
			DEBUG_LOG("[EDSLibrary::lookup_library] Could not find EDS library. You should pass a custum path or check your local installation.")
			return false;
		}

		m_ready = true;
		return true;
	}

	bool EDSLibrary::load_mandatory_entries() {
		return load_default_eds(301);
	}

	bool EDSLibrary::load_default_eds(uint16_t device_profile_number) {

		assert(m_ready);

		std::string path = m_library_path + "/cia/"+std::to_string(device_profile_number)+".eds";
		if (!fs::exists(path)) {
			DEBUG_LOG("[EDSLibrary::load_default_eds] Default EDS file not available: "<<path);
			return false;
		}

		EDSReader reader(m_map);
		bool success = reader.load_file(path);

		if (!success) {
			ERROR("[EDSLibrary::load_default_eds] Loading file not successful.");
			return false;
		}

		success = reader.import_entries();

		if (!success) {
			ERROR("[EDSLibrary::load_default_eds] Importing entries failed.");
			return false;
		}

		return true;

	}

	bool EDSLibrary::load_manufacturer_eds(uint32_t vendor_id, uint32_t product_code, uint32_t revision_number) {
		assert(m_ready);
		
		// check if there is an EDS file for this revision
		std::string path = m_library_path + "/"+std::to_string(vendor_id)+"/"+std::to_string(product_code)+"."+std::to_string(revision_number)+".eds";
		if (!fs::exists(path)) {
			// check if there is a generic EDS file for product
			path = m_library_path + "/"+std::to_string(vendor_id)+"/"+std::to_string(product_code)+".eds";
			if (!fs::exists(path)) {
				DEBUG_LOG("[EDSLibrary::load_manufacturer_eds] Manufacturer device specific EDS file not available: "<<path);
				return false;
			}
		}

		EDSReader reader(m_map);
		bool success = reader.load_file(path);

		if (!success) {
			ERROR("[EDSLibrary::load_manufacturer_eds] Loading file not successful.");
			return false;
		}

		success = reader.import_entries();

		if (!success) {
			ERROR("[EDSLibrary::load_manufacturer_eds] Importing entries failed.");
			return false;
		}

		return true;
		
	}

	bool EDSLibrary::ready() const {
		return m_ready;
	}

} // end namespace kaco