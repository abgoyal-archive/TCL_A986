

/****************************************************************************/
/****************************************************************************/
#ifndef _DMACHW_H
#define _DMACHW_H

#include <stddef.h>

#include <csp/stdint.h>
#include <mach/csp/dmacHw_reg.h>

#define dmacHw_MAKE_CHANNEL_ID(m, c)         (m << 8 | c)

typedef enum {
	dmacHw_CHANNEL_PRIORITY_0 = dmacHw_REG_CFG_LO_CH_PRIORITY_0,	/* Channel priority 0. Lowest priority DMA channel */
	dmacHw_CHANNEL_PRIORITY_1 = dmacHw_REG_CFG_LO_CH_PRIORITY_1,	/* Channel priority 1 */
	dmacHw_CHANNEL_PRIORITY_2 = dmacHw_REG_CFG_LO_CH_PRIORITY_2,	/* Channel priority 2 */
	dmacHw_CHANNEL_PRIORITY_3 = dmacHw_REG_CFG_LO_CH_PRIORITY_3,	/* Channel priority 3 */
	dmacHw_CHANNEL_PRIORITY_4 = dmacHw_REG_CFG_LO_CH_PRIORITY_4,	/* Channel priority 4 */
	dmacHw_CHANNEL_PRIORITY_5 = dmacHw_REG_CFG_LO_CH_PRIORITY_5,	/* Channel priority 5 */
	dmacHw_CHANNEL_PRIORITY_6 = dmacHw_REG_CFG_LO_CH_PRIORITY_6,	/* Channel priority 6 */
	dmacHw_CHANNEL_PRIORITY_7 = dmacHw_REG_CFG_LO_CH_PRIORITY_7	/* Channel priority 7. Highest priority DMA channel */
} dmacHw_CHANNEL_PRIORITY_e;

/* Source destination master interface */
typedef enum {
	dmacHw_SRC_MASTER_INTERFACE_1 = dmacHw_REG_CTL_SMS_1,	/* Source DMA master interface 1 */
	dmacHw_SRC_MASTER_INTERFACE_2 = dmacHw_REG_CTL_SMS_2,	/* Source DMA master interface 2 */
	dmacHw_DST_MASTER_INTERFACE_1 = dmacHw_REG_CTL_DMS_1,	/* Destination DMA master interface 1 */
	dmacHw_DST_MASTER_INTERFACE_2 = dmacHw_REG_CTL_DMS_2	/* Destination DMA master interface 2 */
} dmacHw_MASTER_INTERFACE_e;

typedef enum {
	dmacHw_SRC_TRANSACTION_WIDTH_8 = dmacHw_REG_CTL_SRC_TR_WIDTH_8,	/* Source 8 bit  (1 byte) per transaction */
	dmacHw_SRC_TRANSACTION_WIDTH_16 = dmacHw_REG_CTL_SRC_TR_WIDTH_16,	/* Source 16 bit (2 byte) per transaction */
	dmacHw_SRC_TRANSACTION_WIDTH_32 = dmacHw_REG_CTL_SRC_TR_WIDTH_32,	/* Source 32 bit (4 byte) per transaction */
	dmacHw_SRC_TRANSACTION_WIDTH_64 = dmacHw_REG_CTL_SRC_TR_WIDTH_64,	/* Source 64 bit (8 byte) per transaction */
	dmacHw_DST_TRANSACTION_WIDTH_8 = dmacHw_REG_CTL_DST_TR_WIDTH_8,	/* Destination 8 bit  (1 byte) per transaction */
	dmacHw_DST_TRANSACTION_WIDTH_16 = dmacHw_REG_CTL_DST_TR_WIDTH_16,	/* Destination 16 bit (2 byte) per transaction */
	dmacHw_DST_TRANSACTION_WIDTH_32 = dmacHw_REG_CTL_DST_TR_WIDTH_32,	/* Destination 32 bit (4 byte) per transaction */
	dmacHw_DST_TRANSACTION_WIDTH_64 = dmacHw_REG_CTL_DST_TR_WIDTH_64	/* Destination 64 bit (8 byte) per transaction */
} dmacHw_TRANSACTION_WIDTH_e;

typedef enum {
	dmacHw_SRC_BURST_WIDTH_0 = dmacHw_REG_CTL_SRC_MSIZE_0,	/* Source No burst */
	dmacHw_SRC_BURST_WIDTH_4 = dmacHw_REG_CTL_SRC_MSIZE_4,	/* Source 4  X dmacHw_TRANSACTION_WIDTH_xxx bytes per burst */
	dmacHw_SRC_BURST_WIDTH_8 = dmacHw_REG_CTL_SRC_MSIZE_8,	/* Source 8  X dmacHw_TRANSACTION_WIDTH_xxx bytes per burst */
	dmacHw_SRC_BURST_WIDTH_16 = dmacHw_REG_CTL_SRC_MSIZE_16,	/* Source 16 X dmacHw_TRANSACTION_WIDTH_xxx bytes per burst */
	dmacHw_DST_BURST_WIDTH_0 = dmacHw_REG_CTL_DST_MSIZE_0,	/* Destination No burst */
	dmacHw_DST_BURST_WIDTH_4 = dmacHw_REG_CTL_DST_MSIZE_4,	/* Destination 4  X dmacHw_TRANSACTION_WIDTH_xxx bytes per burst */
	dmacHw_DST_BURST_WIDTH_8 = dmacHw_REG_CTL_DST_MSIZE_8,	/* Destination 8  X dmacHw_TRANSACTION_WIDTH_xxx bytes per burst */
	dmacHw_DST_BURST_WIDTH_16 = dmacHw_REG_CTL_DST_MSIZE_16	/* Destination 16 X dmacHw_TRANSACTION_WIDTH_xxx bytes per burst */
} dmacHw_BURST_WIDTH_e;

typedef enum {
	dmacHw_TRANSFER_TYPE_MEM_TO_MEM = dmacHw_REG_CTL_TTFC_MM_DMAC,	/* Memory to memory transfer */
	dmacHw_TRANSFER_TYPE_PERIPHERAL_TO_MEM = dmacHw_REG_CTL_TTFC_PM_DMAC,	/* Peripheral to memory transfer */
	dmacHw_TRANSFER_TYPE_MEM_TO_PERIPHERAL = dmacHw_REG_CTL_TTFC_MP_DMAC,	/* Memory to peripheral transfer */
	dmacHw_TRANSFER_TYPE_PERIPHERAL_TO_PERIPHERAL = dmacHw_REG_CTL_TTFC_PP_DMAC	/* Peripheral to peripheral transfer */
} dmacHw_TRANSFER_TYPE_e;

typedef enum {
	dmacHw_TRANSFER_MODE_PERREQUEST,	/* Block transfer per DMA request */
	dmacHw_TRANSFER_MODE_CONTINUOUS,	/* Continuous transfer of streaming data */
	dmacHw_TRANSFER_MODE_PERIODIC	/* Periodic transfer of streaming data */
} dmacHw_TRANSFER_MODE_e;

typedef enum {
	dmacHw_SRC_ADDRESS_UPDATE_MODE_INC = dmacHw_REG_CTL_SINC_INC,	/* Increment source address after every transaction */
	dmacHw_SRC_ADDRESS_UPDATE_MODE_DEC = dmacHw_REG_CTL_SINC_DEC,	/* Decrement source address after every transaction */
	dmacHw_DST_ADDRESS_UPDATE_MODE_INC = dmacHw_REG_CTL_DINC_INC,	/* Increment destination address after every transaction */
	dmacHw_DST_ADDRESS_UPDATE_MODE_DEC = dmacHw_REG_CTL_DINC_DEC,	/* Decrement destination address after every transaction */
	dmacHw_SRC_ADDRESS_UPDATE_MODE_NC = dmacHw_REG_CTL_SINC_NC,	/* No change in source address after every transaction */
	dmacHw_DST_ADDRESS_UPDATE_MODE_NC = dmacHw_REG_CTL_DINC_NC	/* No change in destination address after every transaction */
} dmacHw_ADDRESS_UPDATE_MODE_e;

typedef enum {
	dmacHw_FLOW_CONTROL_DMA,	/* DMA working as flow controller (default) */
	dmacHw_FLOW_CONTROL_PERIPHERAL	/* Peripheral working as flow controller */
} dmacHw_FLOW_CONTROL_e;

typedef enum {
	dmacHw_TRANSFER_STATUS_BUSY,	/* DMA Transfer ongoing */
	dmacHw_TRANSFER_STATUS_DONE,	/* DMA Transfer completed */
	dmacHw_TRANSFER_STATUS_ERROR	/* DMA Transfer error */
} dmacHw_TRANSFER_STATUS_e;

typedef enum {
	dmacHw_INTERRUPT_DISABLE,	/* Interrupt disable  */
	dmacHw_INTERRUPT_ENABLE	/* Interrupt enable */
} dmacHw_INTERRUPT_e;

typedef enum {
	dmacHw_INTERRUPT_STATUS_NONE = 0x0,	/* No DMA interrupt */
	dmacHw_INTERRUPT_STATUS_TRANS = 0x1,	/* End of DMA transfer interrupt */
	dmacHw_INTERRUPT_STATUS_BLOCK = 0x2,	/* End of block transfer interrupt */
	dmacHw_INTERRUPT_STATUS_ERROR = 0x4	/* Error interrupt */
} dmacHw_INTERRUPT_STATUS_e;

typedef enum {
	dmacHw_CONTROLLER_ATTRIB_CHANNEL_NUM,	/* Number of DMA channel */
	dmacHw_CONTROLLER_ATTRIB_CHANNEL_MAX_BLOCK_SIZE,	/* Maximum channel burst size */
	dmacHw_CONTROLLER_ATTRIB_MASTER_INTF_NUM,	/* Number of DMA master interface */
	dmacHw_CONTROLLER_ATTRIB_CHANNEL_BUS_WIDTH,	/* Channel Data bus width */
	dmacHw_CONTROLLER_ATTRIB_CHANNEL_FIFO_SIZE	/* Channel FIFO size */
} dmacHw_CONTROLLER_ATTRIB_e;

typedef unsigned long dmacHw_HANDLE_t;	/* DMA channel handle */
typedef uint32_t dmacHw_ID_t;	/* DMA channel Id.  Must be created using
				   "dmacHw_MAKE_CHANNEL_ID" macro
				 */
/* DMA channel configuration parameters */
typedef struct {
	uint32_t srcPeripheralPort;	/* Source peripheral port */
	uint32_t dstPeripheralPort;	/* Destination peripheral port */
	uint32_t srcStatusRegisterAddress;	/* Source status register address */
	uint32_t dstStatusRegisterAddress;	/* Destination status register address of type  */

	uint32_t srcGatherWidth;	/* Number of bytes gathered before successive gather opearation */
	uint32_t srcGatherJump;	/* Number of bytes jumpped before successive gather opearation */
	uint32_t dstScatterWidth;	/* Number of bytes sacattered before successive scatter opearation */
	uint32_t dstScatterJump;	/* Number of bytes jumpped  before successive scatter opearation */
	uint32_t maxDataPerBlock;	/* Maximum number of bytes to be transferred per block/descrptor.
					   0 = Maximum possible.
					 */

	dmacHw_ADDRESS_UPDATE_MODE_e srcUpdate;	/* Source address update mode */
	dmacHw_ADDRESS_UPDATE_MODE_e dstUpdate;	/* Destination address update mode */
	dmacHw_TRANSFER_TYPE_e transferType;	/* DMA transfer type  */
	dmacHw_TRANSFER_MODE_e transferMode;	/* DMA transfer mode */
	dmacHw_MASTER_INTERFACE_e srcMasterInterface;	/* DMA source interface  */
	dmacHw_MASTER_INTERFACE_e dstMasterInterface;	/* DMA destination interface */
	dmacHw_TRANSACTION_WIDTH_e srcMaxTransactionWidth;	/* Source transaction width   */
	dmacHw_TRANSACTION_WIDTH_e dstMaxTransactionWidth;	/* Destination transaction width */
	dmacHw_BURST_WIDTH_e srcMaxBurstWidth;	/* Source burst width */
	dmacHw_BURST_WIDTH_e dstMaxBurstWidth;	/* Destination burst width */
	dmacHw_INTERRUPT_e blockTransferInterrupt;	/* Block trsnafer interrupt */
	dmacHw_INTERRUPT_e completeTransferInterrupt;	/* Complete DMA trsnafer interrupt */
	dmacHw_INTERRUPT_e errorInterrupt;	/* Error interrupt */
	dmacHw_CHANNEL_PRIORITY_e channelPriority;	/* Channel priority */
	dmacHw_FLOW_CONTROL_e flowControler;	/* Data flow controller */
} dmacHw_CONFIG_t;

/****************************************************************************/
/****************************************************************************/
void dmacHw_initDma(void);

/****************************************************************************/
/****************************************************************************/
void dmacHw_exitDma(void);

/****************************************************************************/
/****************************************************************************/
dmacHw_HANDLE_t dmacHw_getChannelHandle(dmacHw_ID_t channelId	/* [ IN ] DMA Channel Id */
    );

/****************************************************************************/
/****************************************************************************/
int dmacHw_initChannel(dmacHw_HANDLE_t handle	/*  [ IN ] DMA Channel handle  */
    );

/****************************************************************************/
/****************************************************************************/
int dmacHw_calculateDescriptorCount(dmacHw_CONFIG_t *pConfig,	/*   [ IN ] Configuration settings */
				    void *pSrcAddr,	/*   [ IN ] Source (Peripheral/Memory) address */
				    void *pDstAddr,	/*   [ IN ] Destination (Peripheral/Memory) address */
				    size_t dataLen	/*   [ IN ] Data length in bytes */
    );

/****************************************************************************/
/****************************************************************************/
int dmacHw_initDescriptor(void *pDescriptorVirt,	/*  [ IN ] Virtual address of uncahced buffer allocated to form descriptor ring */
			  uint32_t descriptorPhyAddr,	/*  [ IN ] Physical address of pDescriptorVirt (descriptor buffer) */
			  uint32_t len,	/*  [ IN ] Size of the pBuf */
			  uint32_t num	/*  [ IN ] Number of descriptor in the ring */
    );

/****************************************************************************/
/****************************************************************************/
uint32_t dmacHw_descriptorLen(uint32_t descCnt	/*  [ IN ] Number of descriptor in the ring */
    );

/****************************************************************************/
/****************************************************************************/
int dmacHw_configChannel(dmacHw_HANDLE_t handle,	/*  [ IN ] DMA Channel handle  */
			 dmacHw_CONFIG_t *pConfig	/*   [ IN ] Configuration settings */
    );

/****************************************************************************/
/****************************************************************************/
int dmacHw_setDataDescriptor(dmacHw_CONFIG_t *pConfig,	/*  [ IN ] Configuration settings */
			     void *pDescriptor,	/*  [ IN ] Descriptor buffer  */
			     void *pSrcAddr,	/*  [ IN ] Source (Peripheral/Memory) address */
			     void *pDstAddr,	/*  [ IN ] Destination (Peripheral/Memory) address */
			     size_t dataLen	/*  [ IN ] Length in bytes   */
    );

/****************************************************************************/
/****************************************************************************/
dmacHw_TRANSFER_STATUS_e dmacHw_transferCompleted(dmacHw_HANDLE_t handle	/*   [ IN ] DMA Channel handle  */
    );

/****************************************************************************/
/****************************************************************************/
int dmacHw_setControlDescriptor(dmacHw_CONFIG_t *pConfig,	/*  [ IN ] Configuration settings */
				void *pDescriptor,	/*  [ IN ] Descriptor buffer  */
				uint32_t ctlAddress,	/*  [ IN ] Address of the device control register  */
				uint32_t control	/*  [ IN ] Device control information */
    );

/****************************************************************************/
/****************************************************************************/
int dmacHw_readTransferredData(dmacHw_HANDLE_t handle,	/*  [ IN ] DMA Channel handle    */
			       dmacHw_CONFIG_t *pConfig,	/*  [ IN ]  Configuration settings */
			       void *pDescriptor,	/*  [ IN ] Descriptor buffer  */
			       void **ppBbuf,	/*  [ OUT ] Data received */
			       size_t *pLlen	/*  [ OUT ] Length of the data received */
    );

/****************************************************************************/
/****************************************************************************/
int dmacHw_setVariableDataDescriptor(dmacHw_HANDLE_t handle,	/*  [ IN ] DMA Channel handle   */
				     dmacHw_CONFIG_t *pConfig,	/*  [ IN ] Configuration settings */
				     void *pDescriptor,	/*  [ IN ] Descriptor buffer  */
				     uint32_t srcAddr,	/*  [ IN ] Source peripheral address */
				     void *(*fpAlloc) (int len),	/*  [ IN ] Function pointer  that provides destination memory */
				     int len,	/*  [ IN ] Number of bytes "fpAlloc" will allocate for destination */
				     int num	/*  [ IN ] Number of descriptor to set */
    );

/****************************************************************************/
/****************************************************************************/
void dmacHw_initiateTransfer(dmacHw_HANDLE_t handle,	/*   [ IN ] DMA Channel handle */
			     dmacHw_CONFIG_t *pConfig,	/*   [ IN ] Configuration settings */
			     void *pDescriptor	/*   [ IN ] Descriptor buffer  */
    );

/****************************************************************************/
/****************************************************************************/
void dmacHw_resetDescriptorControl(void *pDescriptor	/*   [ IN ] Descriptor buffer  */
    );

/****************************************************************************/
/****************************************************************************/
void dmacHw_stopTransfer(dmacHw_HANDLE_t handle	/*   [ IN ] DMA Channel handle */
    );

/****************************************************************************/
/****************************************************************************/
uint32_t dmacHw_descriptorPending(dmacHw_HANDLE_t handle,	/*   [ IN ] DMA Channel handle */
				  void *pDescriptor	/*   [ IN ] Descriptor buffer */
    );

/****************************************************************************/
/****************************************************************************/
int dmacHw_freeMem(dmacHw_CONFIG_t *pConfig,	/*  [ IN ] Configuration settings */
		   void *pDescriptor,	/*  [ IN ] Descriptor buffer  */
		   void (*fpFree) (void *)	/*  [ IN ] Function pointer to free data memory */
    );

/****************************************************************************/
/****************************************************************************/
void dmacHw_clearInterrupt(dmacHw_HANDLE_t handle	/*  [ IN ] DMA Channel handle  */
    );

/****************************************************************************/
/****************************************************************************/
dmacHw_INTERRUPT_STATUS_e dmacHw_getInterruptStatus(dmacHw_HANDLE_t handle	/*  [ IN ] DMA Channel handle  */
    );

/****************************************************************************/
/****************************************************************************/
dmacHw_HANDLE_t dmacHw_getInterruptSource(void);

/****************************************************************************/
/****************************************************************************/
void dmacHw_setChannelUserData(dmacHw_HANDLE_t handle,	/*  [ IN ] DMA Channel handle  */
			       void *userData	/*  [ IN ] User data  */
    );

/****************************************************************************/
/****************************************************************************/
void *dmacHw_getChannelUserData(dmacHw_HANDLE_t handle	/*  [ IN ] DMA Channel handle  */
    );

/****************************************************************************/
/****************************************************************************/
void dmacHw_printDebugInfo(dmacHw_HANDLE_t handle,	/*  [ IN ] DMA Channel handle  */
			   void *pDescriptor,	/*  [ IN ] Descriptor buffer  */
			   int (*fpPrint) (const char *, ...)	/*  [ IN ] Print callback function */
    );

/****************************************************************************/
/****************************************************************************/
uint32_t dmacHw_getDmaControllerAttribute(dmacHw_HANDLE_t handle,	/*  [ IN ]  DMA Channel handle  */
					  dmacHw_CONTROLLER_ATTRIB_e attr	/*  [ IN ]  DMA Controler attribute of type  dmacHw_CONTROLLER_ATTRIB_e */
    );

#endif /* _DMACHW_H */
