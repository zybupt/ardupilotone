#ifndef AP_Var_keys_H
#define AP_Var_keys_H

enum keys {
	k_config = 0,
	k_cntrl,
	k_chMode,

#if VEHICLE_TYPE == CAR
	k_pidThr,
	k_pidStr,
	k_chThr,
	k_chStr,
#elif VEHICLE_TYPE == QUAD
	k_pidPN,
	k_pidPE,
	k_pidPD,
	k_pidRoll,
	k_pidPitch,
	k_pidYaw,
	k_chLeft,
	k_chRight,
	k_chFront,
	k_chBack,
#endif // QUAD

	// 226-256 allocated for commands
	// (31 commands max, @ 33 bytes/command -> 1023/1024 bytes of EEPROM used
	k_firstCommand=225,
};

// max 256 keys

#endif
