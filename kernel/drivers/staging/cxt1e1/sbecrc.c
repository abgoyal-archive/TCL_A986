

#include <linux/types.h>
#include "pmcc4_sysdep.h"
#include "sbecom_inline_linux.h"
#include "sbe_promformat.h"

/* defines */
#define CRC32_POLYNOMIAL                0xEDB88320L
#define CRC_TABLE_ENTRIES                       256



static      u_int32_t crcTableInit;

#ifdef STATIC_CRC_TABLE
static u_int32_t CRCTable[CRC_TABLE_ENTRIES];

#endif



static void
genCrcTable (u_int32_t *CRCTable)
{
    int         ii, jj;
    u_int32_t      crc;

    for (ii = 0; ii < CRC_TABLE_ENTRIES; ii++)
    {
        crc = ii;
        for (jj = 8; jj > 0; jj--)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ CRC32_POLYNOMIAL;
            else
                crc >>= 1;
        }
        CRCTable[ii] = crc;
    }

    crcTableInit++;
}



void
sbeCrc (u_int8_t *buffer,          /* data buffer to crc */
        u_int32_t count,           /* length of block in bytes */
        u_int32_t initialCrc,      /* starting CRC */
        u_int32_t *result)
{
    u_int32_t     *tbl = 0;
    u_int32_t      temp1, temp2, crc;

    /*
     * if table not yet created, do so. Don't care about "extra" time
     * checking this everytime sbeCrc() is called, since CRC calculations are
     * already time consuming
     */
    if (!crcTableInit)
    {
#ifdef STATIC_CRC_TABLE
        tbl = &CRCTable;
        genCrcTable (tbl);
#else
        tbl = (u_int32_t *) OS_kmalloc (CRC_TABLE_ENTRIES * sizeof (u_int32_t));
        if (tbl == 0)
        {
            *result = 0;            /* dummy up return value due to malloc
                                     * failure */
            return;
        }
        genCrcTable (tbl);
#endif
    }
    /* inverting bits makes ZMODEM & PKZIP compatible */
    crc = initialCrc ^ 0xFFFFFFFFL;

    while (count-- != 0)
    {
        temp1 = (crc >> 8) & 0x00FFFFFFL;
        temp2 = tbl[((int) crc ^ *buffer++) & 0xff];
        crc = temp1 ^ temp2;
    }

    crc ^= 0xFFFFFFFFL;

    *result = crc;

#ifndef STATIC_CRC_TABLE
    crcTableInit = 0;
    OS_kfree (tbl);
#endif
}

/*** End-of-File ***/
