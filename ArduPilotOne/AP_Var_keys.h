#ifndef AP_Var_keys_H
#define AP_Var_keys_H

#include "config.h"

enum keys {
	k_config = 0,
	k_cntrl,
	k_chMode,

#if VEHICLE_TYPE == VEHICLE_CAR
	k_pidThr,
	k_pidStr,
	k_chThr,
	k_chStr,
#elif VEHICLE_TYPE == VEHICLE_QUAD
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

	// 200-256 allocated for commands
	k_firstCommand=200,
};

// max 256 keys

#endif
