

/****************************************************************************/
/****************************************************************************/

#if !defined(ASM_ARM_ARCH_BCMRING_DMA_H)
#define ASM_ARM_ARCH_BCMRING_DMA_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/kernel.h>
#include <linux/wait.h>
#include <linux/semaphore.h>
#include <csp/dmacHw.h>
#include <mach/timer.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>

/* ---- Constants and Types ---------------------------------------------- */

/* If DMA_DEBUG_TRACK_RESERVATION is set to a non-zero value, then the filename */
/* and line number of the reservation request will be recorded in the channel table */

#define DMA_DEBUG_TRACK_RESERVATION   1

#define DMA_NUM_CONTROLLERS     2
#define DMA_NUM_CHANNELS        8	/* per controller */

typedef enum {
	DMA_DEVICE_MEM_TO_MEM,	/* For memory to memory transfers */
	DMA_DEVICE_I2S0_DEV_TO_MEM,
	DMA_DEVICE_I2S0_MEM_TO_DEV,
	DMA_DEVICE_I2S1_DEV_TO_MEM,
	DMA_DEVICE_I2S1_MEM_TO_DEV,
	DMA_DEVICE_APM_CODEC_A_DEV_TO_MEM,
	DMA_DEVICE_APM_CODEC_A_MEM_TO_DEV,
	DMA_DEVICE_APM_CODEC_B_DEV_TO_MEM,
	DMA_DEVICE_APM_CODEC_B_MEM_TO_DEV,
	DMA_DEVICE_APM_CODEC_C_DEV_TO_MEM,	/* Additional mic input for beam-forming */
	DMA_DEVICE_APM_PCM0_DEV_TO_MEM,
	DMA_DEVICE_APM_PCM0_MEM_TO_DEV,
	DMA_DEVICE_APM_PCM1_DEV_TO_MEM,
	DMA_DEVICE_APM_PCM1_MEM_TO_DEV,
	DMA_DEVICE_SPUM_DEV_TO_MEM,
	DMA_DEVICE_SPUM_MEM_TO_DEV,
	DMA_DEVICE_SPIH_DEV_TO_MEM,
	DMA_DEVICE_SPIH_MEM_TO_DEV,
	DMA_DEVICE_UART_A_DEV_TO_MEM,
	DMA_DEVICE_UART_A_MEM_TO_DEV,
	DMA_DEVICE_UART_B_DEV_TO_MEM,
	DMA_DEVICE_UART_B_MEM_TO_DEV,
	DMA_DEVICE_PIF_MEM_TO_DEV,
	DMA_DEVICE_PIF_DEV_TO_MEM,
	DMA_DEVICE_ESW_DEV_TO_MEM,
	DMA_DEVICE_ESW_MEM_TO_DEV,
	DMA_DEVICE_VPM_MEM_TO_MEM,
	DMA_DEVICE_CLCD_MEM_TO_MEM,
	DMA_DEVICE_NAND_MEM_TO_MEM,
	DMA_DEVICE_MEM_TO_VRAM,
	DMA_DEVICE_VRAM_TO_MEM,

	/* Add new entries before this line. */

	DMA_NUM_DEVICE_ENTRIES,
	DMA_DEVICE_NONE = 0xff,	/* Special value to indicate that no device is currently assigned. */

} DMA_Device_t;


#define DMA_INVALID_HANDLE  ((DMA_Handle_t) -1)

typedef int DMA_Handle_t;


typedef struct {
	void *virtAddr;		/* Virtual Address of the descriptor ring */
	dma_addr_t physAddr;	/* Physical address of the descriptor ring */
	int descriptorsAllocated;	/* Number of descriptors allocated in the descriptor ring */
	size_t bytesAllocated;	/* Number of bytes allocated in the descriptor ring */

} DMA_DescriptorRing_t;


#define DMA_MEM_MAP_MIN_SIZE    4096	/* Pages less than this size are better */
					/* off not being DMA'd. */

typedef enum {
	DMA_MEM_TYPE_NONE,	/* Not a valid setting */
	DMA_MEM_TYPE_VMALLOC,	/* Memory came from vmalloc call */
	DMA_MEM_TYPE_KMALLOC,	/* Memory came from kmalloc call */
	DMA_MEM_TYPE_DMA,	/* Memory came from dma_alloc_xxx call */
	DMA_MEM_TYPE_USER,	/* Memory came from user space. */

} DMA_MemType_t;

/* A segment represents a physically and virtually contiguous chunk of memory. */
/* i.e. each segment can be DMA'd */
/* A user of the DMA code will add memory regions. Each region may need to be */
/* represented by one or more segments. */

typedef struct {
	void *virtAddr;		/* Virtual address used for this segment */
	dma_addr_t physAddr;	/* Physical address this segment maps to */
	size_t numBytes;	/* Size of the segment, in bytes */

} DMA_Segment_t;

/* A region represents a virtually contiguous chunk of memory, which may be */
/* made up of multiple segments. */

typedef struct {
	DMA_MemType_t memType;
	void *virtAddr;
	size_t numBytes;

	/* Each region (virtually contiguous) consists of one or more segments. Each */
	/* segment is virtually and physically contiguous. */

	int numSegmentsUsed;
	int numSegmentsAllocated;
	DMA_Segment_t *segment;

	/* When a region corresponds to user memory, we need to lock all of the pages */
	/* down before we can figure out the physical addresses. The lockedPage array contains */
	/* the pages that were locked, and which subsequently need to be unlocked once the */
	/* memory is unmapped. */

	unsigned numLockedPages;
	struct page **lockedPages;

} DMA_Region_t;

typedef struct {
	int inUse;		/* Is this mapping currently being used? */
	struct semaphore lock;	/* Acquired when using this structure */
	enum dma_data_direction dir;	/* Direction this transfer is intended for */

	/* In the event that we're mapping user memory, we need to know which task */
	/* the memory is for, so that we can obtain the correct mm locks. */

	struct task_struct *userTask;

	int numRegionsUsed;
	int numRegionsAllocated;
	DMA_Region_t *region;

} DMA_MemMap_t;


/* The device handler is called whenever a DMA operation completes. The reaon */
/* for it to be called will be a bitmask with one or more of the following bits */
/* set. */

#define DMA_HANDLER_REASON_BLOCK_COMPLETE       dmacHw_INTERRUPT_STATUS_BLOCK
#define DMA_HANDLER_REASON_TRANSFER_COMPLETE    dmacHw_INTERRUPT_STATUS_TRANS
#define DMA_HANDLER_REASON_ERROR                dmacHw_INTERRUPT_STATUS_ERROR

typedef void (*DMA_DeviceHandler_t) (DMA_Device_t dev, int reason,
				     void *userData);

#define DMA_DEVICE_FLAG_ON_DMA0             0x00000001
#define DMA_DEVICE_FLAG_ON_DMA1             0x00000002
#define DMA_DEVICE_FLAG_PORT_PER_DMAC       0x00000004	/* If set, it means that the port used on DMAC0 is different from the port used on DMAC1 */
#define DMA_DEVICE_FLAG_ALLOC_DMA1_FIRST    0x00000008	/* If set, allocate from DMA1 before allocating from DMA0 */
#define DMA_DEVICE_FLAG_IS_DEDICATED        0x00000100
#define DMA_DEVICE_FLAG_NO_ISR              0x00000200
#define DMA_DEVICE_FLAG_ALLOW_LARGE_FIFO    0x00000400
#define DMA_DEVICE_FLAG_IN_USE              0x00000800	/* If set, device is in use on a channel */

/* Note: Some DMA devices can be used from multiple DMA Controllers. The bitmask is used to */
/*       determine which DMA controllers a given device can be used from, and the interface */
/*       array determeines the actual interface number to use for a given controller. */

typedef struct {
	uint32_t flags;		/* Bitmask of DMA_DEVICE_FLAG_xxx constants */
	uint8_t dedicatedController;	/* Controller number to use if DMA_DEVICE_FLAG_IS_DEDICATED is set. */
	uint8_t dedicatedChannel;	/* Channel number to use if DMA_DEVICE_FLAG_IS_DEDICATED is set. */
	const char *name;	/* Will show up in the /proc entry */

	uint32_t dmacPort[DMA_NUM_CONTROLLERS];	/* Specifies the port number when DMA_DEVICE_FLAG_PORT_PER_DMAC flag is set */

	dmacHw_CONFIG_t config;	/* Configuration to use when DMA'ing using this device */

	void *userData;		/* Passed to the devHandler */
	DMA_DeviceHandler_t devHandler;	/* Called when DMA operations finish. */

	timer_tick_count_t transferStartTime;	/* Time the current transfer was started */

	/* The following statistical information will be collected and presented in a proc entry. */
	/* Note: With a contiuous bandwidth of 1 Gb/sec, it would take 584 years to overflow */
	/*       a 64 bit counter. */

	uint64_t numTransfers;	/* Number of DMA transfers performed */
	uint64_t transferTicks;	/* Total time spent doing DMA transfers (measured in timer_tick_count_t's) */
	uint64_t transferBytes;	/* Total bytes transferred */
	uint32_t timesBlocked;	/* Number of times a channel was unavailable */
	uint32_t numBytes;	/* Last transfer size */

	/* It's not possible to free memory which is allocated for the descriptors from within */
	/* the ISR. So make the presumption that a given device will tend to use the */
	/* same sized buffers over and over again, and we keep them around. */

	DMA_DescriptorRing_t ring;	/* Ring of descriptors allocated for this device */

	/* We stash away some of the information from the previous transfer. If back-to-back */
	/* transfers are performed from the same buffer, then we don't have to keep re-initializing */
	/* the descriptor buffers. */

	uint32_t prevNumBytes;
	dma_addr_t prevSrcData;
	dma_addr_t prevDstData;

} DMA_DeviceAttribute_t;



#define DMA_CHANNEL_FLAG_IN_USE         0x00000001
#define DMA_CHANNEL_FLAG_IS_DEDICATED   0x00000002
#define DMA_CHANNEL_FLAG_NO_ISR         0x00000004
#define DMA_CHANNEL_FLAG_LARGE_FIFO     0x00000008

typedef struct {
	uint32_t flags;		/* bitmask of DMA_CHANNEL_FLAG_xxx constants */
	DMA_Device_t devType;	/* Device this channel is currently reserved for */
	DMA_Device_t lastDevType;	/* Device type that used this previously */
	char name[20];		/* Name passed onto request_irq */

#if (DMA_DEBUG_TRACK_RESERVATION)
	const char *fileName;	/* Place where channel reservation took place */
	int lineNum;		/* Place where channel reservation took place */
#endif
	dmacHw_HANDLE_t dmacHwHandle;	/* low level channel handle. */

} DMA_Channel_t;


typedef struct {
	DMA_Channel_t channel[DMA_NUM_CHANNELS];

} DMA_Controller_t;


typedef struct {
	struct semaphore lock;	/* acquired when manipulating table entries */
	wait_queue_head_t freeChannelQ;

	DMA_Controller_t controller[DMA_NUM_CONTROLLERS];

} DMA_Global_t;

/* ---- Variable Externs ------------------------------------------------- */

extern DMA_DeviceAttribute_t DMA_gDeviceAttribute[DMA_NUM_DEVICE_ENTRIES];

/* ---- Function Prototypes ---------------------------------------------- */

#if defined(__KERNEL__)

/****************************************************************************/
/****************************************************************************/

int dma_init(void);

#if (DMA_DEBUG_TRACK_RESERVATION)
DMA_Handle_t dma_request_channel_dbg(DMA_Device_t dev, const char *fileName,
				     int lineNum);
#define dma_request_channel(dev)  dma_request_channel_dbg(dev, __FILE__, __LINE__)
#else

/****************************************************************************/
/****************************************************************************/

DMA_Handle_t dma_request_channel(DMA_Device_t dev	/* Device to use with the allocated channel. */
    );
#endif

/****************************************************************************/
/****************************************************************************/

int dma_free_channel(DMA_Handle_t channel	/* DMA handle. */
    );

/****************************************************************************/
/****************************************************************************/

int dma_device_is_channel_shared(DMA_Device_t dev	/* Device to check. */
    );

/****************************************************************************/
/****************************************************************************/

int dma_alloc_descriptor_ring(DMA_DescriptorRing_t *ring,	/* Descriptor ring to populate */
			      int numDescriptors	/* Number of descriptors that need to be allocated. */
    );

/****************************************************************************/
/****************************************************************************/

void dma_free_descriptor_ring(DMA_DescriptorRing_t *ring	/* Descriptor to release */
    );

/****************************************************************************/
/****************************************************************************/

int dma_init_descriptor_ring(DMA_DescriptorRing_t *ring,	/* Descriptor ring to initialize */
			     int numDescriptors	/* Number of descriptors to initialize. */
    );

/****************************************************************************/
/****************************************************************************/

int dma_calculate_descriptor_count(DMA_Device_t device,	/* DMA Device that this will be associated with */
				   dma_addr_t srcData,	/* Place to get data to write to device */
				   dma_addr_t dstData,	/* Pointer to device data address */
				   size_t numBytes	/* Number of bytes to transfer to the device */
    );

/****************************************************************************/
/****************************************************************************/

int dma_add_descriptors(DMA_DescriptorRing_t *ring,	/* Descriptor ring to add descriptors to */
			DMA_Device_t device,	/* DMA Device that descriptors are for */
			dma_addr_t srcData,	/* Place to get data (memory or device) */
			dma_addr_t dstData,	/* Place to put data (memory or device) */
			size_t numBytes	/* Number of bytes to transfer to the device */
    );

/****************************************************************************/
/****************************************************************************/

int dma_set_device_descriptor_ring(DMA_Device_t device,	/* Device to update the descriptor ring for. */
				   DMA_DescriptorRing_t *ring	/* Descriptor ring to add descriptors to */
    );

/****************************************************************************/
/****************************************************************************/

int dma_get_device_descriptor_ring(DMA_Device_t device,	/* Device to retrieve the descriptor ring for. */
				   DMA_DescriptorRing_t *ring	/* Place to store retrieved ring */
    );

/****************************************************************************/
/****************************************************************************/

int dma_alloc_descriptors(DMA_Handle_t handle,	/* DMA Handle */
			  dmacHw_TRANSFER_TYPE_e transferType,	/* Type of transfer being performed */
			  dma_addr_t srcData,	/* Place to get data to write to device */
			  dma_addr_t dstData,	/* Pointer to device data address */
			  size_t numBytes	/* Number of bytes to transfer to the device */
    );

/****************************************************************************/
/****************************************************************************/

int dma_alloc_double_dst_descriptors(DMA_Handle_t handle,	/* DMA Handle */
				     dma_addr_t srcData,	/* Physical address of source data */
				     dma_addr_t dstData1,	/* Physical address of first destination buffer */
				     dma_addr_t dstData2,	/* Physical address of second destination buffer */
				     size_t numBytes	/* Number of bytes in each destination buffer */
    );

/****************************************************************************/
/****************************************************************************/

int dma_init_mem_map(DMA_MemMap_t *memMap	/* Stores state information about the map */
    );

/****************************************************************************/
/****************************************************************************/

int dma_term_mem_map(DMA_MemMap_t *memMap	/* Stores state information about the map */
    );

/****************************************************************************/
/****************************************************************************/

DMA_MemType_t dma_mem_type(void *addr);

/****************************************************************************/
/****************************************************************************/

static inline void dma_mem_map_set_user_task(DMA_MemMap_t *memMap,
					     struct task_struct *task)
{
	memMap->userTask = task;
}

/****************************************************************************/
/****************************************************************************/

int dma_mem_supports_dma(void *addr);

/****************************************************************************/
/****************************************************************************/

int dma_map_start(DMA_MemMap_t *memMap,	/* Stores state information about the map */
		  enum dma_data_direction dir	/* Direction that the mapping will be going */
    );

/****************************************************************************/
/****************************************************************************/

int dma_map_add_region(DMA_MemMap_t *memMap,	/* Stores state information about the map */
		       void *mem,	/* Virtual address that we want to get a map of */
		       size_t numBytes	/* Number of bytes being mapped */
    );

/****************************************************************************/
/****************************************************************************/

int dma_map_create_descriptor_ring(DMA_Device_t dev,	/* DMA device (where the ring is stored) */
				   DMA_MemMap_t *memMap,	/* Memory map that will be used */
				   dma_addr_t devPhysAddr	/* Physical address of device */
    );

/****************************************************************************/
/****************************************************************************/

int dma_map_mem(DMA_MemMap_t *memMap,	/* Stores state information about the map */
		void *addr,	/* Virtual address that we want to get a map of */
		size_t count,	/* Number of bytes being mapped */
		enum dma_data_direction dir	/* Direction that the mapping will be going */
    );

/****************************************************************************/
/****************************************************************************/

int dma_unmap(DMA_MemMap_t *memMap,	/* Stores state information about the map */
	      int dirtied	/* non-zero if any of the pages were modified */
    );

/****************************************************************************/
/****************************************************************************/

int dma_start_transfer(DMA_Handle_t handle);

/****************************************************************************/
/****************************************************************************/

int dma_stop_transfer(DMA_Handle_t handle);

/****************************************************************************/
/****************************************************************************/

int dma_wait_transfer_done(DMA_Handle_t handle);

/****************************************************************************/
/****************************************************************************/

int dma_transfer(DMA_Handle_t handle,	/* DMA Handle */
		 dmacHw_TRANSFER_TYPE_e transferType,	/* Type of transfer being performed */
		 dma_addr_t srcData,	/* Place to get data to write to device */
		 dma_addr_t dstData,	/* Pointer to device data address */
		 size_t numBytes	/* Number of bytes to transfer to the device */
    );

/****************************************************************************/
/****************************************************************************/

static inline int dma_transfer_to_device(DMA_Handle_t handle,	/* DMA Handle */
					 dma_addr_t srcData,	/* Place to get data to write to device (physical address) */
					 dma_addr_t dstData,	/* Pointer to device data address (physical address) */
					 size_t numBytes	/* Number of bytes to transfer to the device */
    ) {
	return dma_transfer(handle,
			    dmacHw_TRANSFER_TYPE_MEM_TO_PERIPHERAL,
			    srcData, dstData, numBytes);
}

/****************************************************************************/
/****************************************************************************/

static inline int dma_transfer_from_device(DMA_Handle_t handle,	/* DMA Handle */
					   dma_addr_t srcData,	/* Pointer to the device data address (physical address) */
					   dma_addr_t dstData,	/* Place to store data retrieved from the device (physical address) */
					   size_t numBytes	/* Number of bytes to retrieve from the device */
    ) {
	return dma_transfer(handle,
			    dmacHw_TRANSFER_TYPE_PERIPHERAL_TO_MEM,
			    srcData, dstData, numBytes);
}

/****************************************************************************/
/****************************************************************************/

static inline int dma_transfer_mem_to_mem(DMA_Handle_t handle,	/* DMA Handle */
					  dma_addr_t srcData,	/* Place to transfer data from (physical address) */
					  dma_addr_t dstData,	/* Place to transfer data to (physical address) */
					  size_t numBytes	/* Number of bytes to transfer */
    ) {
	return dma_transfer(handle,
			    dmacHw_TRANSFER_TYPE_MEM_TO_MEM,
			    srcData, dstData, numBytes);
}

/****************************************************************************/
/****************************************************************************/

int dma_set_device_handler(DMA_Device_t dev,	/* Device to set the callback for. */
			   DMA_DeviceHandler_t devHandler,	/* Function to call when the DMA completes */
			   void *userData	/* Pointer which will be passed to devHandler. */
    );

#endif

#endif /* ASM_ARM_ARCH_BCMRING_DMA_H */
