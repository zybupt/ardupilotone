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
#endif // CAR

	// 200-256 allocated for commands
	k_cmdStart=250,
};

// max 256 keys

#endif
