
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include "partition_define.h"


/*=======================================================================*/
/* NAND PARTITION Mapping                                                  */
/*=======================================================================*/
#ifdef CONFIG_MTD_PARTITIONS

struct mtd_partition g_pasStatic_Partition[] = {
    { 
        .name   = "preloader",
        .offset = 0x0,
        .size   = PART_SIZE_PRELOADER,
        .mask_flags = MTD_WRITEABLE,
    },
    { 
	.name   = "dsp_bl",
	.offset = MTDPART_OFS_APPEND,
	.size   = PART_SIZE_DSP_BL,
	.mask_flags = MTD_WRITEABLE,
    },
    { 
        .name   = "pro_info",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_PRO_INFO,
    },
    { 
        .name   = "nvram",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_NVRAM,
    },
    { 
        .name   = "seccnfg",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_SECCFG,
        //.mask_flags = MTD_WRITEABLE,
    },          
    { 
        .name   = "uboot",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_UBOOT,
        .mask_flags = MTD_WRITEABLE,
    },     
    { 
        .name   = "boot",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_BOOTIMG,        
    },
    { 
        .name   = "recovery",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_RECOVERY,
    },
    { 
        .name   = "secstatic",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_SEC_RO,
    },      
    { 
        .name   = "misc",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_MISC,
    },
    { 
        .name   = "logo",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_LOGO,
        //.mask_flags = MTD_WRITEABLE,
    },    
    { 
        .name   = "expdb",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_EXPDB,
    },    
    { 
        .name   = "system",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_ANDROID,
    },
    { 
        .name   = "custpack",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_CUSTPACK,
    },
    
    {
        .name   = "watchdog",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_WATCHDOG,
    },
    { 
        .name   = "cache",
        .offset = MTDPART_OFS_APPEND,
        .size   = PART_SIZE_CACHE,
    },    

    { 
        .name   = "userdata",
        .offset = MTDPART_OFS_APPEND,
        .size   = MTDPART_SIZ_FULL,
    },   
};

#define NUM_PARTITIONS ARRAY_SIZE(g_pasStatic_Partition)
int part_num = NUM_PARTITIONS;
#endif
