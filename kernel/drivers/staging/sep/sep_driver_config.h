

#ifndef __SEP_DRIVER_CONFIG_H__
#define __SEP_DRIVER_CONFIG_H__



#define SEP_DRIVER_POLLING_MODE                         1

#define SEP_DRIVER_RECONFIG_MESSAGE_AREA                0

/* the mode for running on the ARM1172 Evaluation platform (flag is 1) */
#define SEP_DRIVER_ARM_DEBUG_MODE                       0


/* flag for the input array */
#define SEP_DRIVER_IN_FLAG                              0

/* flag for output array */
#define SEP_DRIVER_OUT_FLAG                             1

/* maximum number of entries in one LLI tables */
#define SEP_DRIVER_ENTRIES_PER_TABLE_IN_SEP             8





#define SEP_DRIVER_MAX_MESSAGE_SIZE_IN_BYTES                  (8 * 1024)

/* the size of the message shared area in pages */
#define SEP_DRIVER_MESSAGE_SHARED_AREA_SIZE_IN_BYTES          (8 * 1024)

/* the size of the data pool static area in pages */
#define SEP_DRIVER_STATIC_AREA_SIZE_IN_BYTES                  (4 * 1024)

/* the size of the data pool shared area size in pages */
#define SEP_DRIVER_DATA_POOL_SHARED_AREA_SIZE_IN_BYTES        (12 * 1024)

/* the size of the message shared area in pages */
#define SEP_DRIVER_SYNCHRONIC_DMA_TABLES_AREA_SIZE_IN_BYTES   (1024 * 5)


/* the size of the data pool shared area size in pages */
#define SEP_DRIVER_FLOW_DMA_TABLES_AREA_SIZE_IN_BYTES         (1024 * 4)

/* system data (time, caller id etc') pool */
#define SEP_DRIVER_SYSTEM_DATA_MEMORY_SIZE_IN_BYTES           100


#define SEP_DRIVER_MMMAP_AREA_SIZE                            (1024 * 24)



/* message area offset */
#define SEP_DRIVER_MESSAGE_AREA_OFFSET_IN_BYTES               0

/* static pool area offset */
#define SEP_DRIVER_STATIC_AREA_OFFSET_IN_BYTES \
		(SEP_DRIVER_MESSAGE_SHARED_AREA_SIZE_IN_BYTES)

/* data pool area offset */
#define SEP_DRIVER_DATA_POOL_AREA_OFFSET_IN_BYTES \
	(SEP_DRIVER_STATIC_AREA_OFFSET_IN_BYTES + \
	SEP_DRIVER_STATIC_AREA_SIZE_IN_BYTES)

/* synhronic dma tables area offset */
#define SEP_DRIVER_SYNCHRONIC_DMA_TABLES_AREA_OFFSET_IN_BYTES \
	(SEP_DRIVER_DATA_POOL_AREA_OFFSET_IN_BYTES + \
	SEP_DRIVER_DATA_POOL_SHARED_AREA_SIZE_IN_BYTES)

/* sep driver flow dma tables area offset */
#define SEP_DRIVER_FLOW_DMA_TABLES_AREA_OFFSET_IN_BYTES \
	(SEP_DRIVER_SYNCHRONIC_DMA_TABLES_AREA_OFFSET_IN_BYTES + \
	SEP_DRIVER_SYNCHRONIC_DMA_TABLES_AREA_SIZE_IN_BYTES)

/* system memory offset in bytes */
#define SEP_DRIVER_SYSTEM_DATA_MEMORY_OFFSET_IN_BYTES \
	(SEP_DRIVER_FLOW_DMA_TABLES_AREA_OFFSET_IN_BYTES + \
	SEP_DRIVER_FLOW_DMA_TABLES_AREA_SIZE_IN_BYTES)

/* offset of the time area */
#define SEP_DRIVER_SYSTEM_TIME_MEMORY_OFFSET_IN_BYTES \
	(SEP_DRIVER_SYSTEM_DATA_MEMORY_OFFSET_IN_BYTES)



/* start physical address of the SEP registers memory in HOST */
#define SEP_IO_MEM_REGION_START_ADDRESS                       0x80000000

/* size of the SEP registers memory region  in HOST (for now 100 registers) */
#define SEP_IO_MEM_REGION_SIZE                                (2 * 0x100000)

/* define the number of IRQ for SEP interrupts */
#define SEP_DIRVER_IRQ_NUM                                    1

/* maximum number of add buffers */
#define SEP_MAX_NUM_ADD_BUFFERS                               100

/* number of flows */
#define SEP_DRIVER_NUM_FLOWS                                  4

/* maximum number of entries in flow table */
#define SEP_DRIVER_MAX_FLOW_NUM_ENTRIES_IN_TABLE              25

/* offset of the num entries in the block length entry of the LLI */
#define SEP_NUM_ENTRIES_OFFSET_IN_BITS                        24

/* offset of the interrupt flag in the block length entry of the LLI */
#define SEP_INT_FLAG_OFFSET_IN_BITS                           31

/* mask for extracting data size from LLI */
#define SEP_TABLE_DATA_SIZE_MASK                              0xFFFFFF

/* mask for entries after being shifted left */
#define SEP_NUM_ENTRIES_MASK                                  0x7F

/* default flow id */
#define SEP_FREE_FLOW_ID                                      0xFFFFFFFF

#define SEP_TEMP_FLOW_ID                   (SEP_DRIVER_NUM_FLOWS + 1)

/* maximum add buffers message length in bytes */
#define SEP_MAX_ADD_MESSAGE_LENGTH_IN_BYTES                   (7 * 4)

/* maximum number of concurrent virtual buffers */
#define SEP_MAX_VIRT_BUFFERS_CONCURRENT                       100

/* the token that defines the start of time address */
#define SEP_TIME_VAL_TOKEN                                    0x12345678

/* DEBUG LEVEL MASKS */
#define SEP_DEBUG_LEVEL_BASIC       0x1

#define SEP_DEBUG_LEVEL_EXTENDED    0x4


/* Debug helpers */

#define dbg(fmt, args...) \
do {\
	if (debug & SEP_DEBUG_LEVEL_BASIC) \
		printk(KERN_DEBUG fmt, ##args); \
} while(0);

#define edbg(fmt, args...) \
do { \
	if (debug & SEP_DEBUG_LEVEL_EXTENDED) \
		printk(KERN_DEBUG fmt, ##args); \
} while(0);



#endif
