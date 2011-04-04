/*
 * AP_MavlinkCommand.h
 *
 *  Created on: Apr 4, 2011
 *      Author: jgoppert
 */

#ifndef AP_MAVLINKCOMMAND_H_
#define AP_MAVLINKCOMMAND_H_

class AP_MavlinkCommand {
private:
	struct CommandStorage {
		MAV_CMD command;
		bool autocontinue;
		MAV_FRAME frame;
		float param1;
		float param2;
		float param3;
		float param4;
		float x;
		float y;
		float z;
	};
	AP_VarS<CommandStorage> _data;
	uint8_t _seq;
public:
	/**
	 * Constructor for loading from memory.
	 * @param index Start at zero.
	 */
	AP_MavlinkCommand(uint8_t index) :
		_data(k_firstCommand + index) {
		_data.load();
		_seq = index;
	}

	/**
	 * Constructor for saving from command a mavlink waypoint.
	 * @param cmd The mavlink_waopint_t structure for the command.
	 */
	AP_MavlinkCommand(mavlink_waypoint_t cmd) :
		_data(k_firstCommand + cmd.seq), _seq(cmd.seq) {
		setCommand(MAV_CMD(cmd.command));
		setAutocontinue(cmd.autocontinue);
		setFrame((MAV_FRAME) cmd.frame);
		setParam1(cmd.param1);
		setParam2(cmd.param2);
		setParam3(cmd.param3);
		setParam4(cmd.param4);
		setX(cmd.x);
		setY(cmd.y);
		setZ(cmd.z);
		_data.save();
	}
	bool save() {
		return _data.save();
	}
	bool load() {
		return _data.load();
	}
	uint8_t getSeq() {
		return _seq;
	}
	bool getAutocontinue() {
		return _data.get().autocontinue;
	}
	void setAutocontinue(bool val) {
		_data.get().autocontinue = val;
	}
	void setSeq(uint8_t val) {
		_seq = val;
	}
	MAV_CMD getCommand() {
		return _data.get().command;
	}
	void setCommand(MAV_CMD val) {
		_data.get().command = val;
	}
	uint8_t getFrame() {
		return _data.get().frame;
	}
	void setFrame(MAV_FRAME val) {
		_data.get().frame = val;
	}
	uint8_t getParam1() {
		return _data.get().param1;
	}
	void setParam1(float val) {
		_data.get().param1 = val;
	}
	uint8_t getParam2() {
		return _data.get().param2;
	}
	void setParam2(float val) {
		_data.get().param2 = val;
	}
	uint8_t getParam3() {
		return _data.get().param3;
	}
	void setParam3(float val) {
		_data.get().param3 = val;
	}
	uint8_t getParam4() {
		return _data.get().param4;
	}
	void setParam4(float val) {
		_data.get().param4 = val;
	}
	uint8_t getX() {
		return _data.get().x;
	}
	void setX(float val) {
		_data.get().x = val;
	}
	uint8_t getY() {
		return _data.get().y;
	}
	void setY(float val) {
		_data.get().y = val;
	}
	uint8_t getZ() {
		return _data.get().z;
	}
	void setZ(float val) {
		_data.get().z = val;
	}
	bool getCurrent() {
		return (currentIndex.get() == getSeq());
	}
	float getLat() {
		switch (getFrame()) {
		case MAV_FRAME_GLOBAL:
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
			return getX();
			break;
		case MAV_FRAME_LOCAL:
		case MAV_FRAME_MISSION:
			return 0;
			break;
		}
	}
	void setLat(float val) {
		switch (getFrame()) {
		case MAV_FRAME_GLOBAL:
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
			setY(val);
			break;
		case MAV_FRAME_LOCAL:
			setY(val + AP_MavlinkCommand(0).getLat());
			break;
		case MAV_FRAME_MISSION:
			break;
		}
	}
	float getLon() {
		switch (getFrame()) {
		case MAV_FRAME_GLOBAL:
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
			return getX();
			break;
		case MAV_FRAME_LOCAL:
		case MAV_FRAME_MISSION:
			return 0;
			break;
		}
	}
	void setLon(float val) {
		switch (getFrame()) {
		case MAV_FRAME_GLOBAL:
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
			setX(val);
			break;
		case MAV_FRAME_LOCAL:
			setX(val + AP_MavlinkCommand(0).getLon());
			break;
		case MAV_FRAME_MISSION:
			break;
		}
	}
	float getAlt() {
		switch (getFrame()) {
		case MAV_FRAME_GLOBAL:
			return getZ();
			break;
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
		case MAV_FRAME_LOCAL:
			return getZ() + AP_MavlinkCommand(0).getAlt();
			break;
		case MAV_FRAME_MISSION:
			return 0;
			break;
		}
	}
	/*
	 * get the relative altitude to home in meters
	 */
	float getRelAlt() {
			switch (getFrame()) {
			case MAV_FRAME_GLOBAL:
				return getZ() - AP_MavlinkCommand(0).getAlt();
				break;
			case MAV_FRAME_GLOBAL_RELATIVE_ALT:
			case MAV_FRAME_LOCAL:
				return getZ();
				break;
			case MAV_FRAME_MISSION:
				return 0;
				break;
			}
		}
	/*
	 * set the relative altitude to home in meters
	 */
	void setAlt(float val) {
		switch (getFrame()) {
		case MAV_FRAME_GLOBAL:
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
			setX(val);
			break;
		case MAV_FRAME_LOCAL:
			setX(val + AP_MavlinkCommand(0).getLon());
			break;
		case MAV_FRAME_MISSION:
			break;
		}
	}

	/**
	 * conversion for outbound packets to ground station
	 * @return output the mavlink_waypoint_t packet
	 */
	mavlink_waypoint_t convert() {
		mavlink_waypoint_t mavCmd;
		mavCmd.seq = getSeq();
		mavCmd.command = _data.get().command;
		mavCmd.frame = _data.get().frame;
		mavCmd.param1 = _data.get().param1;
		mavCmd.param2 = _data.get().param2;
		mavCmd.param3 = _data.get().param3;
		mavCmd.param4 = _data.get().param4;
		mavCmd.x = _data.get().x;
		mavCmd.y = _data.get().y;
		mavCmd.z = _data.get().z;
		mavCmd.autocontinue = _data.get().autocontinue;
		mavCmd.current = getCurrent();
		mavCmd.target_component = mavlink_system.compid;
		mavCmd.target_system = mavlink_system.sysid;
		return mavCmd;
	}
	/**
	 * Calculate the bearing from this command to the next command
	 * @param next The command to calculate the bearing to.
	 * @return
	 */
	float bearingTo(AP_MavlinkCommand next) {

	}

	//		deltaLng = latLngInt2Radians(next.lngInt() - lngInt());
	//		cosDeltaLng = cos(deltaLng);
	//		sinDeltaLng = sin(deltaLng);
	//
	//		cosLat = cos(latRad());
	//		sinLat = sin(latRad());
	//
	//		sinNextLat = sin(next.latRad());
	//		cosNextLat = cos(next.latRad());
	//
	//		return atan2(sinDeltaLng * cosNextLat,
	//				cosLat * sinNextLat - sinLat * cosNextLat * cosDeltaLng);
	//           }

	// calculates distance to a location
	//	float distanceTo(AP_MavlinkCommand next) {
	//		deltaLat = latLngInt2Radians(next.latInt() - latInt());
	//		deltaLng = latLngInt2Radians(next.lngInt() - lngInt());
	//
	//		cosLat = cos(latRad());
	//		cosNextLat = cos(next.latRad());
	//
	//		sinDeltaLat2 = sin(deltaLat / 2);
	//		sinDeltaLng2 = sin(deltaLng / 2);
	//
	//		float a = sinDeltaLat2 * sinDeltaLat2 + cosLat * cosNextLat
	//				* sinDeltaLng2 * sinDeltaLng2;
	//		float c = 2 * atan2(sqrt(a), sqrt(1 - a));
	//		return rEarth * c;
	//	}

	// calculates cross track of a current location
	//	float crossTrack() {
	//		float d = previousWaypoint()->distanceTo(currentPosition());
	//		float bCurrent = perviousWaypoint()->bearingTo(currentPosition());
	//		float bNext = previousWaypoint()->bearingTo(nextWaypoint());
	//		return asin(sin(d / rEarth) * sin(bCurrent - bNext)) * rEarth;
	//	}

	// calculates along  track distance of a current location
	//	float alongTrack() {
	//		dXt = crossTrack(prev, next);
	//		float d = previousWaypoint()->distanceTo(currentPosition());
	//		return acos(cos(d / rEarth) / cos(dXt / rEarth)) * rEarth;
	//	}
	// conversions
	static float latLngInt2Radians(int32_t val) {
		return val / 1e7;
	}
	static float alt2Meters(int32_t val) {
		return val / 1e2;
	}
	static AP_Uint8 number;
	static AP_Uint8 currentIndex;
};
AP_Uint8 AP_MavlinkCommand::number = 1;
AP_Uint8 AP_MavlinkCommand::currentIndex = 1;

#endif /* AP_MAVLINKCOMMAND_H_ */
