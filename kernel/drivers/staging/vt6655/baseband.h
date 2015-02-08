

#ifndef __BASEBAND_H__
#define __BASEBAND_H__

#include "ttype.h"
#include "tether.h"
#include "device.h"

/*---------------------  Export Definitions -------------------------*/

//
// Registers in the BASEBAND
//
#define BB_MAX_CONTEXT_SIZE 256


//
// Baseband RF pair definition in eeprom (Bits 6..0)
//



#define PREAMBLE_LONG   0
#define PREAMBLE_SHORT  1


#define F5G             0
#define F2_4G           1

#define TOP_RATE_54M        0x80000000
#define TOP_RATE_48M        0x40000000
#define TOP_RATE_36M        0x20000000
#define TOP_RATE_24M        0x10000000
#define TOP_RATE_18M        0x08000000
#define TOP_RATE_12M        0x04000000
#define TOP_RATE_11M        0x02000000
#define TOP_RATE_9M         0x01000000
#define TOP_RATE_6M         0x00800000
#define TOP_RATE_55M        0x00400000
#define TOP_RATE_2M         0x00200000
#define TOP_RATE_1M         0x00100000


/*---------------------  Export Types  ------------------------------*/

/*---------------------  Export Macros ------------------------------*/

#define BBvClearFOE(dwIoBase)                               \
{                                                           \
    BBbWriteEmbeded(dwIoBase, 0xB1, 0);                     \
}

#define BBvSetFOE(dwIoBase)                                 \
{                                                           \
    BBbWriteEmbeded(dwIoBase, 0xB1, 0x0C);                  \
}


/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/

UINT
BBuGetFrameTime(
    BYTE byPreambleType,
    BYTE byPktType,
    UINT cbFrameLength,
    WORD wRate
    );

void
BBvCaculateParameter (
    PSDevice pDevice,
    UINT cbFrameLength,
    WORD wRate,
    BYTE byPacketType,
    PWORD pwPhyLen,
    PBYTE pbyPhySrv,
    PBYTE pbyPhySgn
    );

BOOL BBbReadEmbeded(DWORD_PTR dwIoBase, BYTE byBBAddr, PBYTE pbyData);
BOOL BBbWriteEmbeded(DWORD_PTR dwIoBase, BYTE byBBAddr, BYTE byData);

void BBvReadAllRegs(DWORD_PTR dwIoBase, PBYTE pbyBBRegs);
void BBvLoopbackOn(PSDevice pDevice);
void BBvLoopbackOff(PSDevice pDevice);
void BBvSetShortSlotTime(PSDevice pDevice);
BOOL BBbIsRegBitsOn(DWORD_PTR dwIoBase, BYTE byBBAddr, BYTE byTestBits);
BOOL BBbIsRegBitsOff(DWORD_PTR dwIoBase, BYTE byBBAddr, BYTE byTestBits);
void BBvSetVGAGainOffset(PSDevice pDevice, BYTE byData);

// VT3253 Baseband
BOOL BBbVT3253Init(PSDevice pDevice);
void BBvSoftwareReset(DWORD_PTR dwIoBase);
void BBvPowerSaveModeON(DWORD_PTR dwIoBase);
void BBvPowerSaveModeOFF(DWORD_PTR dwIoBase);
void BBvSetTxAntennaMode(DWORD_PTR dwIoBase, BYTE byAntennaMode);
void BBvSetRxAntennaMode(DWORD_PTR dwIoBase, BYTE byAntennaMode);
void BBvSetDeepSleep(DWORD_PTR dwIoBase, BYTE byLocalID);
void BBvExitDeepSleep(DWORD_PTR dwIoBase, BYTE byLocalID);

// timer for antenna diversity

void
TimerSQ3CallBack (
    void *hDeviceContext
    );

void
TimerState1CallBack(
    void *hDeviceContext
    );

void BBvAntennaDiversity(PSDevice pDevice, BYTE byRxRate, BYTE bySQ3);
void
BBvClearAntDivSQ3Value (PSDevice pDevice);

#endif // __BASEBAND_H__
