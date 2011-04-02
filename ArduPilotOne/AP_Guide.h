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

namespace apo {
struct CommandStroage {
	uint8_t _command;
	uint8_t _autocontinue;
	uint8_t _frame;
	uint8_t _options;
	uint8_t _param1;
	int32_t _x;
	int32_t _y;
	int32_t _z;
};

class Command : public AP_VarS<CommandStroage> {
public:
	Command(mavlink_waypoint_t cmd) : AP_VarS<CommandStorage>(k_cmdStart + cmd.seq)
	{
		get()._autocontinue = cmd.autocontinue;
		get()._frame = cmd.frame;
		get()._command = cmd.command;
		get()._x = cmd.x;
		get()._y = cmd.y;
		get()._z = cmd.z;
		get()._param1 = cmd.param1;
	}
};

/// Guide class
class AP_Guide {
public:

	AP_Guide(AP_Navigator * navigator) :
		_navigator(navigator), _headingCommand(), _airSpeedCommand(),
				_groundSpeedCommand(), _altitudeCommand() {
	}
	const float * headingCommand() {
		return &_headingCommand;
	}
	const float * airSpeedCommand() {
		return &_airSpeedCommand;
	}
	const float * groundSpeedCommand() {
		return &_groundSpeedCommand;
	}
	const float * altitudeCommand() {
		return &_altitudeCommand;
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

protected:
	uint8_t _cmdNum;
	uint8_t _cmdIndex;
	AP_Navigator * _navigator;
	float _headingCommand;
	float _airSpeedCommand;
	float _groundSpeedCommand;
	float _altitudeCommand;
	Command _prevCommand;
	Command _nextCommand;
};

class MavlinkGuide: public AP_Guide {
public:

	MavlinkGuide(AP_Navigator * navigator) :
		AP_Guide(navigator) {
	}
	virtual void update() {
		//float dXt = position()->crossTrack(previousWaypoint(),nextWaypoint());
		//float dAt = position()->alongTrack(previousWaypoint(),nextWaypoint());
		//float d = previousWaypoint()->distanceTo(currentPosition());

		//if (d < nextWaypoint()->radius())
		//{
		//currentWaypointIndex++;

		//}
	}
private:
	//AP_Waypoint * currentWaypoint() { return flightPlan[currentWaypointIndex]; }
	//AP_Waypoint * previousWaypoint() { return flightPlan[previousWaypointIndex]; }
	//AP_Waypoint * position() { return _navigator->getPosition(); }
};

} // namespace apo

#endif // AP_Guide_H
// vim:ts=4:sw=4:expandtab
