

#include "rc4.h"

void rc4_init(PRC4Ext pRC4, PBYTE pbyKey, UINT cbKey_len)
{
    UINT  ust1, ust2;
    UINT  keyindex;
    UINT  stateindex;
    PBYTE pbyst;
    UINT  idx;

    pbyst = pRC4->abystate;
    pRC4->ux = 0;
    pRC4->uy = 0;
    for (idx = 0; idx < 256; idx++)
        pbyst[idx] = (BYTE)idx;
    keyindex = 0;
    stateindex = 0;
    for (idx = 0; idx < 256; idx++) {
        ust1 = pbyst[idx];
        stateindex = (stateindex + pbyKey[keyindex] + ust1) & 0xff;
        ust2 = pbyst[stateindex];
        pbyst[stateindex] = (BYTE)ust1;
        pbyst[idx] = (BYTE)ust2;
        if (++keyindex >= cbKey_len)
            keyindex = 0;
    }
}

UINT rc4_byte(PRC4Ext pRC4)
{
    UINT ux;
    UINT uy;
    UINT ustx, usty;
    PBYTE pbyst;

    pbyst = pRC4->abystate;
    ux = (pRC4->ux + 1) & 0xff;
    ustx = pbyst[ux];
    uy = (ustx + pRC4->uy) & 0xff;
    usty = pbyst[uy];
    pRC4->ux = ux;
    pRC4->uy = uy;
    pbyst[uy] = (BYTE)ustx;
    pbyst[ux] = (BYTE)usty;

    return pbyst[(ustx + usty) & 0xff];
}

void rc4_encrypt(PRC4Ext pRC4, PBYTE pbyDest,
                     PBYTE pbySrc, UINT cbData_len)
{
    UINT ii;
    for (ii = 0; ii < cbData_len; ii++)
        pbyDest[ii] = (BYTE)(pbySrc[ii] ^ rc4_byte(pRC4));
}
