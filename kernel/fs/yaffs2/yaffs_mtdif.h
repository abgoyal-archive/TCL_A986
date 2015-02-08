

#ifndef __YAFFS_MTDIF_H__
#define __YAFFS_MTDIF_H__

#include "yaffs_guts.h"

#if (MTD_VERSION_CODE < MTD_VERSION(2, 6, 18))
extern struct nand_oobinfo yaffs_oobinfo;
extern struct nand_oobinfo yaffs_noeccinfo;
#endif
int nandmtd_EraseBlockInNAND(yaffs_Device *dev, int blockNumber);
int nandmtd_InitialiseNAND(yaffs_Device *dev);
#endif
