

#include "upc.h"
#include "tmacro.h"
#include "tether.h"
#include "mac.h"
#include "srom.h"

/*---------------------  Static Definitions -------------------------*/

/*---------------------  Static Classes  ----------------------------*/

/*---------------------  Static Variables  --------------------------*/

/*---------------------  Static Functions  --------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/




BYTE SROMbyReadEmbedded(DWORD_PTR dwIoBase, BYTE byContntOffset)
{
    WORD    wDelay, wNoACK;
    BYTE    byWait;
    BYTE    byData;
    BYTE    byOrg;

    byData = 0xFF;
    VNSvInPortB(dwIoBase + MAC_REG_I2MCFG, &byOrg);
    /* turn off hardware retry for getting NACK */
    VNSvOutPortB(dwIoBase + MAC_REG_I2MCFG, (byOrg & (~I2MCFG_NORETRY)));
    for (wNoACK = 0; wNoACK < W_MAX_I2CRETRY; wNoACK++) {
        VNSvOutPortB(dwIoBase + MAC_REG_I2MTGID, EEP_I2C_DEV_ID);
        VNSvOutPortB(dwIoBase + MAC_REG_I2MTGAD, byContntOffset);

        /* issue read command */
        VNSvOutPortB(dwIoBase + MAC_REG_I2MCSR, I2MCSR_EEMR);
        /* wait DONE be set */
        for (wDelay = 0; wDelay < W_MAX_TIMEOUT; wDelay++) {
            VNSvInPortB(dwIoBase + MAC_REG_I2MCSR, &byWait);
            if (byWait & (I2MCSR_DONE | I2MCSR_NACK))
                break;
            PCAvDelayByIO(CB_DELAY_LOOP_WAIT);
        }
        if ((wDelay < W_MAX_TIMEOUT) &&
             ( !(byWait & I2MCSR_NACK))) {
            break;
        }
    }
    VNSvInPortB(dwIoBase + MAC_REG_I2MDIPT, &byData);
    VNSvOutPortB(dwIoBase + MAC_REG_I2MCFG, byOrg);
    return byData;
}


BOOL SROMbWriteEmbedded(DWORD_PTR dwIoBase, BYTE byContntOffset, BYTE byData)
{
    WORD    wDelay, wNoACK;
    BYTE    byWait;

    BYTE    byOrg;

    VNSvInPortB(dwIoBase + MAC_REG_I2MCFG, &byOrg);
    /* turn off hardware retry for getting NACK */
    VNSvOutPortB(dwIoBase + MAC_REG_I2MCFG, (byOrg & (~I2MCFG_NORETRY)));
    for (wNoACK = 0; wNoACK < W_MAX_I2CRETRY; wNoACK++) {
        VNSvOutPortB(dwIoBase + MAC_REG_I2MTGID, EEP_I2C_DEV_ID);
        VNSvOutPortB(dwIoBase + MAC_REG_I2MTGAD, byContntOffset);
        VNSvOutPortB(dwIoBase + MAC_REG_I2MDOPT, byData);

        /* issue write command */
        VNSvOutPortB(dwIoBase + MAC_REG_I2MCSR, I2MCSR_EEMW);
        /* wait DONE be set */
        for (wDelay = 0; wDelay < W_MAX_TIMEOUT; wDelay++) {
            VNSvInPortB(dwIoBase + MAC_REG_I2MCSR, &byWait);
            if (byWait & (I2MCSR_DONE | I2MCSR_NACK))
                break;
            PCAvDelayByIO(CB_DELAY_LOOP_WAIT);
        }

        if ((wDelay < W_MAX_TIMEOUT) &&
             ( !(byWait & I2MCSR_NACK))) {
            break;
        }
    }
    if (wNoACK == W_MAX_I2CRETRY) {
        VNSvOutPortB(dwIoBase + MAC_REG_I2MCFG, byOrg);
        return FALSE;
    }
    VNSvOutPortB(dwIoBase + MAC_REG_I2MCFG, byOrg);
    return TRUE;
}


void SROMvRegBitsOn(DWORD_PTR dwIoBase, BYTE byContntOffset, BYTE byBits)
{
    BYTE    byOrgData;

    byOrgData = SROMbyReadEmbedded(dwIoBase, byContntOffset);
    SROMbWriteEmbedded(dwIoBase, byContntOffset,(BYTE)(byOrgData | byBits));
}


void SROMvRegBitsOff(DWORD_PTR dwIoBase, BYTE byContntOffset, BYTE byBits)
{
    BYTE    byOrgData;

    byOrgData = SROMbyReadEmbedded(dwIoBase, byContntOffset);
    SROMbWriteEmbedded(dwIoBase, byContntOffset,(BYTE)(byOrgData & (~byBits)));
}


BOOL SROMbIsRegBitsOn(DWORD_PTR dwIoBase, BYTE byContntOffset, BYTE byTestBits)
{
    BYTE    byOrgData;

    byOrgData = SROMbyReadEmbedded(dwIoBase, byContntOffset);
    return (byOrgData & byTestBits) == byTestBits;
}


BOOL SROMbIsRegBitsOff(DWORD_PTR dwIoBase, BYTE byContntOffset, BYTE byTestBits)
{
    BYTE    byOrgData;

    byOrgData = SROMbyReadEmbedded(dwIoBase, byContntOffset);
    return !(byOrgData & byTestBits);
}


void SROMvReadAllContents(DWORD_PTR dwIoBase, PBYTE pbyEepromRegs)
{
    int     ii;

    /* ii = Rom Address */
    for (ii = 0; ii < EEP_MAX_CONTEXT_SIZE; ii++) {
        *pbyEepromRegs = SROMbyReadEmbedded(dwIoBase,(BYTE) ii);
        pbyEepromRegs++;
    }
}


void SROMvWriteAllContents(DWORD_PTR dwIoBase, PBYTE pbyEepromRegs)
{
    int     ii;

    /* ii = Rom Address */
    for (ii = 0; ii < EEP_MAX_CONTEXT_SIZE; ii++) {
        SROMbWriteEmbedded(dwIoBase,(BYTE) ii, *pbyEepromRegs);
        pbyEepromRegs++;
    }
}


void SROMvReadEtherAddress(DWORD_PTR dwIoBase, PBYTE pbyEtherAddress)
{
    BYTE     ii;

    /* ii = Rom Address */
    for (ii = 0; ii < ETH_ALEN; ii++) {
        *pbyEtherAddress = SROMbyReadEmbedded(dwIoBase, ii);
        pbyEtherAddress++;
    }
}


void SROMvWriteEtherAddress(DWORD_PTR dwIoBase, PBYTE pbyEtherAddress)
{
    BYTE     ii;

    /* ii = Rom Address */
    for (ii = 0; ii < ETH_ALEN; ii++) {
        SROMbWriteEmbedded(dwIoBase, ii, *pbyEtherAddress);
        pbyEtherAddress++;
    }
}


void SROMvReadSubSysVenId(DWORD_PTR dwIoBase, PDWORD pdwSubSysVenId)
{
    PBYTE   pbyData;

    pbyData = (PBYTE)pdwSubSysVenId;
    /* sub vendor */
    *pbyData = SROMbyReadEmbedded(dwIoBase, 6);
    *(pbyData+1) = SROMbyReadEmbedded(dwIoBase, 7);
    /* sub system */
    *(pbyData+2) = SROMbyReadEmbedded(dwIoBase, 8);
    *(pbyData+3) = SROMbyReadEmbedded(dwIoBase, 9);
}

BOOL SROMbAutoLoad(DWORD_PTR dwIoBase)
{
    BYTE    byWait;
    int     ii;

    BYTE    byOrg;

    VNSvInPortB(dwIoBase + MAC_REG_I2MCFG, &byOrg);
    /* turn on hardware retry */
    VNSvOutPortB(dwIoBase + MAC_REG_I2MCFG, (byOrg | I2MCFG_NORETRY));

    MACvRegBitsOn(dwIoBase, MAC_REG_I2MCSR, I2MCSR_AUTOLD);

    /* ii = Rom Address */
    for (ii = 0; ii < EEP_MAX_CONTEXT_SIZE; ii++) {
        MACvTimer0MicroSDelay(dwIoBase, CB_EEPROM_READBYTE_WAIT);
        VNSvInPortB(dwIoBase + MAC_REG_I2MCSR, &byWait);
        if ( !(byWait & I2MCSR_AUTOLD))
            break;
    }

    VNSvOutPortB(dwIoBase + MAC_REG_I2MCFG, byOrg);

    if (ii == EEP_MAX_CONTEXT_SIZE)
        return FALSE;
    return TRUE;
}


