

#include "device.h"
#include "tmacro.h"
#include "tcrc.h"
#include "tether.h"

/*---------------------  Static Definitions -------------------------*/

/*---------------------  Static Classes  ----------------------------*/

/*---------------------  Static Variables  --------------------------*/

/*---------------------  Static Functions  --------------------------*/

/*---------------------  Export Variables  --------------------------*/



BYTE ETHbyGetHashIndexByCrc32 (PBYTE pbyMultiAddr)
{
    int     ii;
    BYTE    byTmpHash;
    BYTE    byHash = 0;

    // get the least 6-bits from CRC generator
    byTmpHash = (BYTE)(CRCdwCrc32(pbyMultiAddr, ETH_ALEN,
            0xFFFFFFFFL) & 0x3F);
    // reverse most bit to least bit
    for (ii = 0; ii < (sizeof(byTmpHash) * 8); ii++) {
        byHash <<= 1;
        if (byTmpHash & 0x01)
            byHash |= 1;
        byTmpHash >>= 1;
    }

    // adjust 6-bits to the right most
    return (byHash >> 2);
}


BOOL ETHbIsBufferCrc32Ok (PBYTE pbyBuffer, UINT cbFrameLength)
{
    DWORD dwCRC;

    dwCRC = CRCdwGetCrc32(pbyBuffer, cbFrameLength - 4);
    if (cpu_to_le32(*((PDWORD)(pbyBuffer + cbFrameLength - 4))) != dwCRC) {
        return FALSE;
    }
    return TRUE;
}

