/*
 * AP_Guide.h
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

#ifndef AP_Guide_H
#define AP_Guide_H

#include "GCS_MAVLink.h"
#include "AP_Navigator.h"
#include <AP_Common.h>
#include <AP_Vector.h>
#include "defines.h"
#include "AP_Var_keys.h"
#include "RangeFinder.h"

namespace apo {

class Command {
private:
	struct CommandStorage {
		uint8_t _command;
		uint8_t _autocontinue;
		uint8_t _frame;
		float _param1;
		float _x;
		float _y;
		float _z;
	};
	AP_VarS<CommandStorage> _data;
	static uint8_t _numberOfCommands;
public:
	/**
	 * Constructor for loading from memory.
	 * @param index Start at zero.
	 */
	Command(uint8_t index) :
		_data(k_firstCommand + index) {
		if (getNumberOfCommands() > index)
			_data.load();
	}
	/**
	 * Constructor for saving from command a mavlink waypoint.
	 * @param cmd The mavlink_waopint_t structure for the command.
	 */
	Command(mavlink_waypoint_t cmd) :
		_data(k_firstCommand + cmd.seq) {
		_data.get()._command = cmd.command;
		_data.get()._autocontinue = cmd.autocontinue;
		_data.get()._frame = cmd.frame;
		_data.get()._param1 = cmd.param1;
		_data.get()._x = cmd.x;
		_data.get()._y = cmd.y;
		_data.get()._z = cmd.z;
		_data.save();
	}
	bool save() {
		return _data.save();
	}
	bool load() {
		return _data.load();
	}
	uint8_t getNumberOfCommands() {
		return _numberOfCommands;
	}
	void setNumberOfCommands(uint8_t numberOfCommands) {
		_numberOfCommands = numberOfCommands;
	}
};
uint8_t Command::_numberOfCommands = 1;

/// Guide class
class AP_Guide {
public:

	/**
	 * This is the constructor, which requires a link to the navigator.
	 * @param navigator This is the navigator pointer.
	 */
	AP_Guide(AP_Navigator * navigator) :
		_navigator(navigator), headingCommand(0), airSpeedCommand(0),
				groundSpeedCommand(0), altitudeCommand(0), _prevCommand(0),
				_nextCommand(1) {
	}

	virtual void update() = 0;

	// current command index
	uint8_t cmdIndex() {
		return _cmdIndex;
	}

	// previous command index
	uint8_t prevCmdIndex() {
		if (_cmdIndex = 0)
			return _cmdNum;
		else
			return _cmdIndex - 1;
	}

	// next command index
	uint8_t nextCmdIndex() {
		if (_cmdIndex >= _cmdNum - 1)
			return 0;
		else
			return _cmdIndex + 1;
	}

	float headingCommand;
	float airSpeedCommand;
	float groundSpeedCommand;
	float altitudeCommand;

protected:
	uint8_t _cmdNum;
	uint8_t _cmdIndex;
	AP_Navigator * _navigator;

	Command _prevCommand;
	Command _nextCommand;
};

class MavlinkGuide: public AP_Guide {
public:

	MavlinkGuide(AP_Navigator * navigator,
			RangeFinder * frontRangeFinder =NULL,
			RangeFinder * backRangeFinder =NULL,
			RangeFinder * leftRangeFinder =NULL,
			RangeFinder * rightRangeFinder =NULL) :
			AP_Guide(navigator) ,
			_frontRangeFinder(frontRangeFinder),
			_backRangeFinder(backRangeFinder),
			_leftRangeFinder(leftRangeFinder),
			_rightRangeFinder(leftRangeFinder)
			{
	}
	virtual void update() {

		// stop if your going to drive into something in front of you
		if (_frontRangeFinder && _frontRangeFinder->distance < 10 ) {
			airSpeedCommand = 0;
			groundSpeedCommand = 0;
		}

		//float dXt = position()->crossTrack(previousWaypoint(),nextWaypoint());
		//float dAt = position()->alongTrack(previousWaypoint(),nextWaypoint());
		//float d = previousWaypoint()->distanceTo(currentPosition());

		//if (d < nextWaypoint()->radius())
		//{
		//currentWaypointIndex++;

		//}
	}
private:
	RangeFinder * _frontRangeFinder;
	RangeFinder * _backRangeFinder;
	RangeFinder * _leftRangeFinder;
	RangeFinder * _rightRangeFinder;
};

} // namespace apo

#endif // AP_Guide_H
// vim:ts=4:sw=4:expandtab
