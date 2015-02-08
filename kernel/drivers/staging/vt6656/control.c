

#include "control.h"
#include "rndis.h"

/*---------------------  Static Definitions -------------------------*/
/* static int          msglevel                =MSG_LEVEL_INFO;  */
/* static int          msglevel                =MSG_LEVEL_DEBUG; */
/*---------------------  Static Classes  ----------------------------*/

/*---------------------  Static Variables  --------------------------*/

/*---------------------  Static Functions  --------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/

void ControlvWriteByte(PSDevice pDevice, BYTE byRegType, BYTE byRegOfs,
			BYTE byData)
{
	BYTE	byData1;
	byData1 = byData;
	CONTROLnsRequestOut(pDevice,
		MESSAGE_TYPE_WRITE,
		byRegOfs,
		byRegType,
		1,
		&byData1);
}

void ControlvReadByte(PSDevice pDevice, BYTE byRegType, BYTE byRegOfs,
			PBYTE pbyData)
{
	NTSTATUS	ntStatus;
	BYTE	byData1;
	ntStatus = CONTROLnsRequestIn(pDevice,
					MESSAGE_TYPE_READ,
					byRegOfs,
					byRegType,
					1,
					&byData1);
	*pbyData = byData1;
}

void ControlvMaskByte(PSDevice pDevice, BYTE byRegType, BYTE byRegOfs,
			BYTE byMask, BYTE byData)
{
	BYTE	pbyData[2];
	pbyData[0] = byData;
	pbyData[1] = byMask;
	CONTROLnsRequestOut(pDevice,
				MESSAGE_TYPE_WRITE_MASK,
				byRegOfs,
				byRegType,
				2,
				pbyData);
}
