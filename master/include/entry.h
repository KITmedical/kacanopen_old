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

#include <cstdint>
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <memory>

#include "types.h"
#include "value.h"
#include "access_method.h"

namespace kaco {

	/// \class Entry
	///
	/// This class represents an entry in the object dictionary of a device.
	///
	/// \todo Add missing fields like high and low limit.
	/// \todo Array type entries are currently not used and may be deleted in future.
	class Entry {

	public:

		/// Tag class for distinguishing constructors.
		struct VariableTag {};

		/// Tag class for distinguishing constructors.
		struct ArrayTag {};

		/// Tag for distinguishing constructors.
		static const VariableTag variable_tag;

		/// Tag for distinguishing constructors.
		static const ArrayTag array_tag;

		/// type of a callback for a value changed event
		/// Important: Never call add_value_changed_callback()
		///   from within (-> deadlock)!
		using ValueChangedCallback = std::function< void(const Value& value) >;

		/// Constructs an empty entry.
		Entry();

		/// standard constructor
		Entry(VariableTag tag, uint16_t _index, uint8_t _subindex, std::string _name, Type _type, AccessType _access);

		/// array constructor
		Entry(ArrayTag tag, uint16_t _index, std::string _name, Type _type, AccessType _access);

		/// copy constructor
		Entry(const Entry& other) = delete;

		/// move constructor
		Entry(Entry&& other) = default;

		/// copy assignment
		Entry& operator=(const Entry& other) = delete;

		/// move assignment
		Entry& operator=(Entry&& other) = default;

		/// Sets the value. If the entry is an array, array_index can be specified.
		/// \remark thread-safe
		void set_value(const Value& value, uint8_t array_index=0);

		/// Returns the value. If the entry is an array, array_index can be specified.
		/// \remark thread-safe
		const Value& get_value(uint8_t array_index=0) const;

		/// Returns if the value is set/valid.  If the entry is an array, array_index can be specified.
		/// \remark thread-safe
		bool valid(uint8_t array_index=0) const;

		/// Returns the data type.
		/// \remark thread-safe
		Type get_type() const;

		/// Registers a given function to be called when the value is changed.
		/// \remark thread-safe
		void add_value_changed_callback(ValueChangedCallback callback);

		/// Prints relevant information concerning this entry on standard output - name, index, possibly value, ...
		/// This is used by Device::print_dictionary()
		/// \remark not thread-safe due to std::cout
		void print() const;

		/// Compares entries by index and subindex.
		/// This can be used for sorting the dictionary.
		/// \remark thread-safe
		bool operator<(const Entry& other) const;

		/// index in dictionary
		uint16_t index;

		/// subindex in dictionary.
		/// if is_array==true, this variable is not used
		uint8_t subindex; // only used if is_array==false

		/// Human-readable name
		/// Should be escaped for consitency using Utils::escape().
		std::string name;

		/// Data type of the value.
		Type type;

		/// Accessibility of the entry 
		AccessType access_type;

		/// Distinguishes arrays (index + dynamic array_index) from regular variables (index and subindex)
		bool is_array = false;

		/// Standard method for reading this entry.
		/// Used by Device::get_entry().
		ReadAccessMethod read_access_method = ReadAccessMethod::sdo;

		/// Standard method for writing this entry.
		/// Used by Device::set_entry().
		WriteAccessMethod write_access_method = WriteAccessMethod::sdo;
		
		// maybe supported in future:
		//bool is_slice;
		//uint8_t slice_first_bit;
		//uint8_t slice_last_bit;

		// TODO: Add fields for default, max and min value, pdo mapping boolean, object flags, and object type.
		// TODO: Maybe the array functionality is obsolete when using EDS files...

	private:

		std::vector<Value> m_value;
		std::vector<bool> m_valid;
		Value m_dummy_value;

		std::vector<ValueChangedCallback> m_value_changed_callbacks;
		std::unique_ptr<std::mutex> m_value_changed_callbacks_mutex;

		/// read_write_mutex locks get_value() and set_value() because a PDO
		/// transmitter thread could read a value reference while it is
		/// set by the main thread.
		std::unique_ptr<std::recursive_mutex> m_read_write_mutex; // recursive_mutex because of valid() used by get_entry(), on heap because mutexes aren't movable.


	};

} // end namespace kaco