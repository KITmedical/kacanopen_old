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

#include <map>

#include "entry.h"

namespace kaco {

	/// This class provides access to KaCanOpen's EDS library.
	/// It manages device specific as well as generic CanOpen
	/// dictionaries.
	class EDSLibrary {

	public:

		/// Constructor.
		/// \param target The dictionary, into which entries will be inserted.
		EDSLibrary(std::map<std::string, Entry>& target);

		/// Finds EDS library on disk.
		/// \param path optional custom path to EDS library
		/// \returns true if successful
		bool lookup_library(std::string path = "");

		/// Loads mandatory dictionary entries defined in CiA 301 standard
		/// \returns true if successful
		bool load_mandatory_entries();

		/// Loads entries defined in generic CiA profile EDS files
		/// \param device_profile_number CiA standard profile number
		/// \returns true if successful
		bool load_default_eds(uint16_t device_profile_number);

		/// Loads entries defined in device specific EDS files proviced by manufacturers.
		/// \param vendor_id Vencor ID from identity object in dictionary (one of the mandatory entries)
		/// \param product_code Product code from identity object in dictionary (one of the mandatory entries)
		/// \param revision_number Revision number from identity object in dictionary (one of the mandatory entries)
		/// \returns true if successful
		bool load_manufacturer_eds(uint32_t vendor_id, uint32_t product_code, uint32_t revision_number);

		/// Checks if lookup_library() was successful.
		/// \returns true if ready
		bool ready() const;

	private:

		/// Enable debug logging.
		static const bool debug = true;

		/// reference to the dictionary
		std::map<std::string, Entry>& m_map;

		/// Path to the EDS library in filesystem. Set by lookup_library()
		std::string m_library_path;

		bool m_ready;

	};

} // end namespace kaco