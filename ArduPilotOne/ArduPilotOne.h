/*
 * ArduPilotOne.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ArduPilotOne_H
#define ArduPilotOne_H

#include "APO_Config.h"
#include <AP_Loop.h>
#include "AP_CommLink.h"

/**
 * ArduPilotOne namespace to protect varibles
 * from overlap with avr and libraries etc.
 * ArduPilotOne does not use any global
 * variables.
 */
namespace apo {

// forward declarations
class AP_CommLink;

/**
 * This class encapsulates the entire autopilot system
 * The constructor takes the serial streams available
 * and attaches them to the appropriate devices.
 * Also, since APM_RC is globally instantiated this
 * is also passed to the constructor so that we
 * can avoid and global instance calls for maximum
 * clarity.
 *
 * It inherits from loop to manage
 * the subloops and sets the overal
 * frequency for the autopilot.
 *

 */
class ArduPilotOne: public Loop {
public:
	/**
	 * Default constructor
	 */
	ArduPilotOne(AP_Navigator * navigator, AP_Guide * guide, AP_Controller * controller,
			AP_HardwareAbstractionLayer * hal);

	/**
	 * Accessors
	 */
	AP_Navigator * navigator() {
		return _navigator;
	}
	AP_Guide * guide() {
		return _guide;
	}
	AP_Controller * controller() {
		return _controller;
	}
	AP_HardwareAbstractionLayer * hal() {
		return _hal;
	}

private:

	/**
	 * Loop 0 Callbacks (fastest)
	 * - inertial navigation
	 * @param data A void pointer used to pass the apo class
	 *  so that the apo public interface may be accessed.
	 */
	static void callback0(void * data);

	/**
	 * Loop 1 Callbacks
	 * - control
	 * - compass reading
	 * @see callback0
	 */
	static void callback1(void * data);

	/**
	 * Loop 2 Callbacks
	 * - gps sensor fusion
	 * - compass sensor fusion
	 * @see callback0
	 */
	static void callback2(void * data);

	/**
	 * Loop 3 Callbacks
	 * - slow messages
	 * @see callback0
	 */
	static void callback3(void * data);

	/**
	 * Loop 4 Callbacks
	 * - super slow mesages
	 * - log writing
	 * @see callback0
	 */
	static void callback4(void * data);

	/**
	 * Components
	 */
	AP_Navigator * _navigator;
	AP_Guide * _guide;
	AP_Controller * _controller;
	AP_HardwareAbstractionLayer * _hal;

	/**
	 * Constants
	 */
	static const float deg2rad = M_PI / 180;
	static const float rad2deg = 180 / M_PI;
};

} // namespace apo

#endif // ArduPilotOne_H
