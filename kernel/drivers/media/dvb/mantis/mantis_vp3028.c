

#include "mantis_common.h"
#include "mantis_vp3028.h"

struct zl10353_config mantis_vp3028_config = {
	.demod_address	= 0x0f,
};

#define MANTIS_MODEL_NAME	"VP-3028"
#define MANTIS_DEV_TYPE		"DVB-T"

struct mantis_hwconfig vp3028_mantis_config = {
	.model_name	= MANTIS_MODEL_NAME,
	.dev_type	= MANTIS_DEV_TYPE,
	.ts_size	= MANTIS_TS_188,
	.baud_rate	= MANTIS_BAUD_9600,
	.parity		= MANTIS_PARITY_NONE,
	.bytes		= 0,
};
