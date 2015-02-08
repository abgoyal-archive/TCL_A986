

#ifndef __BFA_DEFS_TSENSOR_H__
#define __BFA_DEFS_TSENSOR_H__

#include <bfa_os_inc.h>
#include <defs/bfa_defs_types.h>

enum bfa_tsensor_status {
	BFA_TSENSOR_STATUS_UNKNOWN   = 1,   /*  unknown status */
	BFA_TSENSOR_STATUS_FAULTY    = 2,   /*  sensor is faulty */
	BFA_TSENSOR_STATUS_BELOW_MIN = 3,   /*  temperature below mininum */
	BFA_TSENSOR_STATUS_NOMINAL   = 4,   /*  normal temperature */
	BFA_TSENSOR_STATUS_ABOVE_MAX = 5,   /*  temperature above maximum */
};

struct bfa_tsensor_attr_s {
	enum bfa_tsensor_status status;	/*  temperature sensor status */
	u32        	value;	/*  current temperature in celsius */
};

#endif /* __BFA_DEFS_TSENSOR_H__ */
