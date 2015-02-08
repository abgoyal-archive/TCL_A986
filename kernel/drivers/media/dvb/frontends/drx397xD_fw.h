

#ifdef _FW_ENTRY
	_FW_ENTRY("drx397xD.A2.fw",	DRXD_FW_A2 = 0,	DRXD_FW_A2	),
	_FW_ENTRY("drx397xD.B1.fw",	DRXD_FW_B1,	DRXD_FW_B1	),
#undef _FW_ENTRY
#endif /* _FW_ENTRY */

#ifdef _BLOB_ENTRY
	_BLOB_ENTRY("InitAtomicRead",	DRXD_InitAtomicRead = 0	),
	_BLOB_ENTRY("InitCE",		DRXD_InitCE		),
	_BLOB_ENTRY("InitCP",		DRXD_InitCP		),
	_BLOB_ENTRY("InitEC",		DRXD_InitEC		),
	_BLOB_ENTRY("InitEQ",		DRXD_InitEQ		),
	_BLOB_ENTRY("InitFE_1",		DRXD_InitFE_1		),
	_BLOB_ENTRY("InitFE_2",		DRXD_InitFE_2		),
	_BLOB_ENTRY("InitFT",		DRXD_InitFT		),
	_BLOB_ENTRY("InitSC",		DRXD_InitSC		),
	_BLOB_ENTRY("ResetCEFR",	DRXD_ResetCEFR		),
	_BLOB_ENTRY("ResetECRAM",	DRXD_ResetECRAM		),
	_BLOB_ENTRY("microcode",	DRXD_microcode		),
#undef _BLOB_ENTRY
#endif /* _BLOB_ENTRY */
