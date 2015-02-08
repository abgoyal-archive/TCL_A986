

#ifndef __IOCTL_H__
#define __IOCTL_H__

#include "device.h"

/*---------------------  Export Definitions -------------------------*/


/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/

int private_ioctl(PSDevice pDevice, struct ifreq *rq);


#endif /* __IOCTL_H__ */
