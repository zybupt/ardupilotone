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

namespace apo
{

/// Guide class
class AP_Guide
{
public:
    //AP_Guide(AP_Navigator * navigator) : _navigator(navigator),
    //_headingCommand(), _airSpeedCommand(), _groundSpeedCommand(),
    //_altitudeCommand()
    //{
    //}
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
protected:
    AP_Navigator * _navigator;
    float _headingCommand;
    float _airSpeedCommand;
    float _groundSpeedCommand;
    float _altitudeCommand;
};

class MavlinkGuide : public AP_Guide
{
public:

    MavlinkGuide() : flightPlan() {
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
    uint8_t currentWaypointIndex;
    uint8_t previousWaypointIndex;
    Vector<mavlink_command_t *> flightPlan;
    void incrementWaypointIndices() {
        currentWaypointIndex++;
        previousWaypointIndex++;
        if (currentWaypointIndex > flightPlan.getSize()) currentWaypointIndex = 0;
        if (previousWaypointIndex > flightPlan.getSize()) previousWaypointIndex = 0;
    }
};

} // namespace apo

#endif // AP_Guide_H

// vim:ts=4:sw=4:expandtab
