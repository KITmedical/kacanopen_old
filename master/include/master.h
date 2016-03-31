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

#include "core.h"
#include "device.h"

#include <vector>
#include <bitset>

namespace kaco {

	/// \class Master
	/// 
	/// This class represents a master node. It listens
	/// for new slaves and provides access to them
	/// via get_slaves().
	class Master {

	public:

		/// Constructor.
		/// Creates Core instance and adds NMT listener for new devices. 
		Master();

		/// Destructor.
		~Master();

		/// Starts master, creates Core and resets all nodes.
		///	\param busname Name of the bus which will be passed to the CAN driver, e.g. slcan0
		///	\param baudrate Baudrate in 1/s, will be passed to the CAN driver, e.g. 500000
		/// \returns true if successful
		bool start(const std::string busname, unsigned baudrate);
		
		/// Stops master and core.
		void stop();

		/// Returns the number of slave devices in the network.
		size_t num_devices() const;

		/// Returns a reference to a slave device object.
		/// \param index Index of the device. Must be smaller than num_devices().
		Device& get_device(size_t index) const;

		/// Core instance.
		Core core;

	private:

		static const bool debug = false;

		std::vector<std::unique_ptr<Device>> m_devices;
		std::bitset<265> m_device_alive;

		NMT::NewDeviceCallback m_new_device_callback_functional;
		bool m_running{false};

		void new_device_callback(uint8_t node_id);

	};

} // end namespace kaco