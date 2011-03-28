/*
 * AP_Location.h
 * Copyright (C) James Goppert 2011 <james.goppert@gmail.com>
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

// NOTE: this won't be used, but there is some useful code left here

#ifndef AP_LOCATION_H
#define AP_LOCATION_H

class AP_Location
{
public:

    // constructor
    AP_Location(float radius = 0) : _radius(radius)
    {
    }

    // calculate bearing
    virtual float bearingTo(AP_Location next) = 0;

    // calculate distance
    virtual float distanceTo(AP_Location next) = 0;

    // calculates cross track of a current location
    virtual float crossTrack(AP_Location prev, AP_Location next) = 0;

    // calculates along  track distance of a current location
    virtual float alongTrack(AP_Location prev, AP_Location next) = 0;

    void setRadius(float radius) { _radius = radius; }
    float getRadius() { return _radius; }

protected:
    // radius, used for transition radius for waypoints
    float _radius;
};

class AP_GlobalLocation : public AP_Location
public:
}

class AP_LocalLocation : public AP_Location
{
public:
    AP_LocalLocation(AP_GeodeticLocation * home) : 
        _home(home)
    {
    }
    void setHome(AP_GeodeticLocation * home) {_home = home; }
    AP_GeodeticLocation getHome() {return AP_GeodeticLocation; }
private:
    AP_GeodeticLocation * _home;
};

class AP_CartesianLocation : AP_LocalLocation
{
public:
    // default constructor
    AP_CartesianLocation(float x, float y, float z, AP_GeodeticLocation * home) :
        AP_LocalLocation(home), _x(x), _y(y), _z(z)
    {
    }
    // calculate bearing
    virtual float bearingTo(AP_Location next)
    {
    }

    // calculate distance
    virtual float distanceTo(AP_Location next)
    {
    }

    // calculates cross track of a current location
    virtual float crossTrack(AP_Location prev, AP_Location next)
    {
    }

    // calculates along  track distance of a current location
    virtual float alongTrack(AP_Location prev, AP_Location next)
    {
    }
}

class AP_GeodeticLocation : AP_GlobalLocation
// current source
// www.movable-type.co.uk/scripts/latlong.html
{
public:
    AP_GeodeticLocation(int32_t lng, int32_t lat, int32_t alt) :
        _lng(lng), _lat(lat), _alt(alt)
    {
    }

    // calculates a bearing to the next waypoint
    float bearingTo(AP_GeodeticLocation next)
    {
        deltaLng = latLngInt2Radians(next.lngInt() - lngInt());
        cosDeltaLng = cos(deltaLng);
        sinDeltaLng = sin(deltaLng);

        cosLat = cos(latRad());
        sinLat = sin(latRad());

        sinNextLat = sin(next.latRad());
        cosNextLat = cos(next.latRad());

        return atan2(sinDeltaLng*cosNextLat,
                cosLat*sinNextLat-sinLat*cosNextLat*cosDeltaLng);
    }

    // calculates distance to a location
    float distanceTo(AP_GeodeticLocation nextLocation)
    {
        deltaLat = latLngInt2Radians(next.latInt() - latInt());
        deltaLng = latLngInt2Radians(next.lngInt() - lngInt());

        cosLat = cos(latRad());
        cosNextLat = cos(next.latRad());

        sinDeltaLat2 = sin(deltaLat/2);
        sinDeltaLng2 = sin(deltaLng/2);

        float a = sinDeltaLat2*sinDeltaLat2 + cosLat*cosNextLat*sinDeltaLng2*sinDeltaLng2;
        float c = 2*atan2(sqrt(a),sqrt(1-a));
        return rEarth * c;
    }

    // calculates cross track of a current location
    virtual float crossTrack(AP_Location prev, AP_Location next)
    {
        float d = previousWaypoint()->distanceTo(currentPosition());
        float bCurrent = perviousWaypoint()->bearingTo(currentPosition());
        float bNext = previousWaypoint()->bearingTo(nextWaypoint());
        return asin(sin(d/rEarth)*sin(bCurrent-bNext))*rEarth;
    }

    // calculates along  track distance of a current location
    virtual float alongTrack(AP_Location prev, AP_Location next)
    {
        dXt = crossTrack(prev,next);
        float d = previousWaypoint()->distanceTo(currentPosition());
        return acos(cos(d/rEarth)/cos(dXt/rEarth))*rEarth;
    }

    // integer accessors, 1 cm precision
    int32_t lngInt() { return _lngInt; }
    int32_t latInt() { return _latInt; }
    int32_t altInt() { return _altInt; }

    // float accessors, 38 cm precision TODO: check this
    float lngRad() { return latLngInt2Radians(_lngInt); }
    float lngDeg() { return lngRad()*rad2Deg; }

    float latRad() { return latLngInt2Radians(_latInt); }
    float latDeg() { return latRad()*rad2Deg; }

    float altM() { return  alt2Meters(_altInt); }

private:
    // constants
    static float rad2Deg = 180.0/M_PI;
    static float rEarth = 6371000;

    // member variables
    int32_t _lngInt; // units of radians * 1e7
    int32_t _latInt; // units  of radians * 1e7
    int32_t _altInt; // units of meters * 1e2

    // trig for integers
    float sinInt(int32_t val) { return sin(val/1e7); }
    float cosInt(int32_t val) { return cos(val/1e7); }
    float tanInt(int32_t val) { return tan(val/1e7); }

    // conversions
    static float latLngInt2Radians(int32_t val) { return val/1e7; } 
    static float alt2Meters(int32_t val) { return val/1e2; } 
};

#endif // AP_LOCATION_H

// vim:ts=4:sw=4:expandtab
