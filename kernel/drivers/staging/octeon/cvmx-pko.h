


#ifndef __CVMX_PKO_H__
#define __CVMX_PKO_H__

#include "cvmx-fpa.h"
#include "cvmx-pow.h"
#include "cvmx-cmd-queue.h"
#include "cvmx-pko-defs.h"

#define CVMX_PKO_COMMAND_BUFFER_SIZE_ADJUST (1)

#define CVMX_PKO_MAX_OUTPUT_QUEUES_STATIC 256
#define CVMX_PKO_MAX_OUTPUT_QUEUES      ((OCTEON_IS_MODEL(OCTEON_CN31XX) || \
	OCTEON_IS_MODEL(OCTEON_CN3010) || OCTEON_IS_MODEL(OCTEON_CN3005) || \
	OCTEON_IS_MODEL(OCTEON_CN50XX)) ? 32 : \
		(OCTEON_IS_MODEL(OCTEON_CN58XX) || \
		OCTEON_IS_MODEL(OCTEON_CN56XX)) ? 256 : 128)
#define CVMX_PKO_NUM_OUTPUT_PORTS       40
/* use this for queues that are not used */
#define CVMX_PKO_MEM_QUEUE_PTRS_ILLEGAL_PID 63
#define CVMX_PKO_QUEUE_STATIC_PRIORITY  9
#define CVMX_PKO_ILLEGAL_QUEUE  0xFFFF
#define CVMX_PKO_MAX_QUEUE_DEPTH 0

typedef enum {
	CVMX_PKO_SUCCESS,
	CVMX_PKO_INVALID_PORT,
	CVMX_PKO_INVALID_QUEUE,
	CVMX_PKO_INVALID_PRIORITY,
	CVMX_PKO_NO_MEMORY,
	CVMX_PKO_PORT_ALREADY_SETUP,
	CVMX_PKO_CMD_QUEUE_INIT_ERROR
} cvmx_pko_status_t;

typedef enum {
	/*
	 * PKO doesn't do any locking. It is the responsibility of the
	 * application to make sure that no other core is accessing
	 * the same queue at the smae time
	 */
	CVMX_PKO_LOCK_NONE = 0,
	/*
	 * PKO performs an atomic tagswitch to insure exclusive access
	 * to the output queue. This will maintain packet ordering on
	 * output.
	 */
	CVMX_PKO_LOCK_ATOMIC_TAG = 1,
	/*
	 * PKO uses the common command queue locks to insure exclusive
	 * access to the output queue. This is a memory based
	 * ll/sc. This is the most portable locking mechanism.
	 */
	CVMX_PKO_LOCK_CMD_QUEUE = 2,
} cvmx_pko_lock_t;

typedef struct {
	uint32_t packets;
	uint64_t octets;
	uint64_t doorbell;
} cvmx_pko_port_status_t;

typedef union {
	uint64_t u64;
	struct {
		/* Must CVMX_IO_SEG */
		uint64_t mem_space:2;
		/* Must be zero */
		uint64_t reserved:13;
		/* Must be one */
		uint64_t is_io:1;
		/* The ID of the device on the non-coherent bus */
		uint64_t did:8;
		/* Must be zero */
		uint64_t reserved2:4;
		/* Must be zero */
		uint64_t reserved3:18;
		/*
		 * The hardware likes to have the output port in
		 * addition to the output queue,
		 */
		uint64_t port:6;
		/*
		 * The output queue to send the packet to (0-127 are
		 * legal)
		 */
		uint64_t queue:9;
		/* Must be zero */
		uint64_t reserved4:3;
	} s;
} cvmx_pko_doorbell_address_t;

typedef union {
	uint64_t u64;
	struct {
		/*
		 * The size of the reg1 operation - could be 8, 16,
		 * 32, or 64 bits.
		 */
		uint64_t size1:2;
		/*
		 * The size of the reg0 operation - could be 8, 16,
		 * 32, or 64 bits.
		 */
		uint64_t size0:2;
		/*
		 * If set, subtract 1, if clear, subtract packet
		 * size.
		 */
		uint64_t subone1:1;
		/*
		 * The register, subtract will be done if reg1 is
		 * non-zero.
		 */
		uint64_t reg1:11;
		/* If set, subtract 1, if clear, subtract packet size */
		uint64_t subone0:1;
		/* The register, subtract will be done if reg0 is non-zero */
		uint64_t reg0:11;
		/*
		 * When set, interpret segment pointer and segment
		 * bytes in little endian order.
		 */
		uint64_t le:1;
		/*
		 * When set, packet data not allocated in L2 cache by
		 * PKO.
		 */
		uint64_t n2:1;
		/*
		 * If set and rsp is set, word3 contains a pointer to
		 * a work queue entry.
		 */
		uint64_t wqp:1;
		/* If set, the hardware will send a response when done */
		uint64_t rsp:1;
		/*
		 * If set, the supplied pkt_ptr is really a pointer to
		 * a list of pkt_ptr's.
		 */
		uint64_t gather:1;
		/*
		 * If ipoffp1 is non zero, (ipoffp1-1) is the number
		 * of bytes to IP header, and the hardware will
		 * calculate and insert the UDP/TCP checksum.
		 */
		uint64_t ipoffp1:7;
		/*
		 * If set, ignore the I bit (force to zero) from all
		 * pointer structures.
		 */
		uint64_t ignore_i:1;
		/*
		 * If clear, the hardware will attempt to free the
		 * buffers containing the packet.
		 */
		uint64_t dontfree:1;
		/*
		 * The total number of segs in the packet, if gather
		 * set, also gather list length.
		 */
		uint64_t segs:6;
		/* Including L2, but no trailing CRC */
		uint64_t total_bytes:16;
	} s;
} cvmx_pko_command_word0_t;

/* CSR typedefs have been moved to cvmx-csr-*.h */

typedef struct {
	/* ptr to start of buffer, offset kept in FAU reg */
	uint64_t *start_ptr;
} cvmx_pko_state_elem_t;

extern void cvmx_pko_initialize_global(void);
extern int cvmx_pko_initialize_local(void);

extern void cvmx_pko_enable(void);

extern void cvmx_pko_disable(void);


extern void cvmx_pko_shutdown(void);

extern cvmx_pko_status_t cvmx_pko_config_port(uint64_t port,
					      uint64_t base_queue,
					      uint64_t num_queues,
					      const uint64_t priority[]);

static inline void cvmx_pko_doorbell(uint64_t port, uint64_t queue,
				     uint64_t len)
{
	cvmx_pko_doorbell_address_t ptr;

	ptr.u64 = 0;
	ptr.s.mem_space = CVMX_IO_SEG;
	ptr.s.did = CVMX_OCT_DID_PKT_SEND;
	ptr.s.is_io = 1;
	ptr.s.port = port;
	ptr.s.queue = queue;
	/*
	 * Need to make sure output queue data is in DRAM before
	 * doorbell write.
	 */
	CVMX_SYNCWS;
	cvmx_write_io(ptr.u64, len);
}


static inline void cvmx_pko_send_packet_prepare(uint64_t port, uint64_t queue,
						cvmx_pko_lock_t use_locking)
{
	if (use_locking == CVMX_PKO_LOCK_ATOMIC_TAG) {
		/*
		 * Must do a full switch here to handle all cases.  We
		 * use a fake WQE pointer, as the POW does not access
		 * this memory.  The WQE pointer and group are only
		 * used if this work is descheduled, which is not
		 * supported by the
		 * cvmx_pko_send_packet_prepare/cvmx_pko_send_packet_finish
		 * combination.  Note that this is a special case in
		 * which these fake values can be used - this is not a
		 * general technique.
		 */
		uint32_t tag =
		    CVMX_TAG_SW_BITS_INTERNAL << CVMX_TAG_SW_SHIFT |
		    CVMX_TAG_SUBGROUP_PKO << CVMX_TAG_SUBGROUP_SHIFT |
		    (CVMX_TAG_SUBGROUP_MASK & queue);
		cvmx_pow_tag_sw_full((cvmx_wqe_t *) cvmx_phys_to_ptr(0x80), tag,
				     CVMX_POW_TAG_TYPE_ATOMIC, 0);
	}
}

static inline cvmx_pko_status_t cvmx_pko_send_packet_finish(
	uint64_t port,
	uint64_t queue,
	cvmx_pko_command_word0_t pko_command,
	union cvmx_buf_ptr packet,
	cvmx_pko_lock_t use_locking)
{
	cvmx_cmd_queue_result_t result;
	if (use_locking == CVMX_PKO_LOCK_ATOMIC_TAG)
		cvmx_pow_tag_sw_wait();
	result = cvmx_cmd_queue_write2(CVMX_CMD_QUEUE_PKO(queue),
				       (use_locking == CVMX_PKO_LOCK_CMD_QUEUE),
				       pko_command.u64, packet.u64);
	if (likely(result == CVMX_CMD_QUEUE_SUCCESS)) {
		cvmx_pko_doorbell(port, queue, 2);
		return CVMX_PKO_SUCCESS;
	} else if ((result == CVMX_CMD_QUEUE_NO_MEMORY)
		   || (result == CVMX_CMD_QUEUE_FULL)) {
		return CVMX_PKO_NO_MEMORY;
	} else {
		return CVMX_PKO_INVALID_QUEUE;
	}
}

static inline cvmx_pko_status_t cvmx_pko_send_packet_finish3(
	uint64_t port,
	uint64_t queue,
	cvmx_pko_command_word0_t pko_command,
	union cvmx_buf_ptr packet,
	uint64_t addr,
	cvmx_pko_lock_t use_locking)
{
	cvmx_cmd_queue_result_t result;
	if (use_locking == CVMX_PKO_LOCK_ATOMIC_TAG)
		cvmx_pow_tag_sw_wait();
	result = cvmx_cmd_queue_write3(CVMX_CMD_QUEUE_PKO(queue),
				       (use_locking == CVMX_PKO_LOCK_CMD_QUEUE),
				       pko_command.u64, packet.u64, addr);
	if (likely(result == CVMX_CMD_QUEUE_SUCCESS)) {
		cvmx_pko_doorbell(port, queue, 3);
		return CVMX_PKO_SUCCESS;
	} else if ((result == CVMX_CMD_QUEUE_NO_MEMORY)
		   || (result == CVMX_CMD_QUEUE_FULL)) {
		return CVMX_PKO_NO_MEMORY;
	} else {
		return CVMX_PKO_INVALID_QUEUE;
	}
}

static inline int cvmx_pko_get_base_queue_per_core(int port, int core)
{
#ifndef CVMX_HELPER_PKO_MAX_PORTS_INTERFACE0
#define CVMX_HELPER_PKO_MAX_PORTS_INTERFACE0 16
#endif
#ifndef CVMX_HELPER_PKO_MAX_PORTS_INTERFACE1
#define CVMX_HELPER_PKO_MAX_PORTS_INTERFACE1 16
#endif

	if (port < CVMX_PKO_MAX_PORTS_INTERFACE0)
		return port * CVMX_PKO_QUEUES_PER_PORT_INTERFACE0 + core;
	else if (port >= 16 && port < 16 + CVMX_PKO_MAX_PORTS_INTERFACE1)
		return CVMX_PKO_MAX_PORTS_INTERFACE0 *
		    CVMX_PKO_QUEUES_PER_PORT_INTERFACE0 + (port -
							   16) *
		    CVMX_PKO_QUEUES_PER_PORT_INTERFACE1 + core;
	else if ((port >= 32) && (port < 36))
		return CVMX_PKO_MAX_PORTS_INTERFACE0 *
		    CVMX_PKO_QUEUES_PER_PORT_INTERFACE0 +
		    CVMX_PKO_MAX_PORTS_INTERFACE1 *
		    CVMX_PKO_QUEUES_PER_PORT_INTERFACE1 + (port -
							   32) *
		    CVMX_PKO_QUEUES_PER_PORT_PCI;
	else if ((port >= 36) && (port < 40))
		return CVMX_PKO_MAX_PORTS_INTERFACE0 *
		    CVMX_PKO_QUEUES_PER_PORT_INTERFACE0 +
		    CVMX_PKO_MAX_PORTS_INTERFACE1 *
		    CVMX_PKO_QUEUES_PER_PORT_INTERFACE1 +
		    4 * CVMX_PKO_QUEUES_PER_PORT_PCI + (port -
							36) *
		    CVMX_PKO_QUEUES_PER_PORT_LOOP;
	else
		/* Given the limit on the number of ports we can map to
		 * CVMX_MAX_OUTPUT_QUEUES_STATIC queues (currently 256,
		 * divided among all cores), the remaining unmapped ports
		 * are assigned an illegal queue number */
		return CVMX_PKO_ILLEGAL_QUEUE;
}

static inline int cvmx_pko_get_base_queue(int port)
{
	return cvmx_pko_get_base_queue_per_core(port, 0);
}

static inline int cvmx_pko_get_num_queues(int port)
{
	if (port < 16)
		return CVMX_PKO_QUEUES_PER_PORT_INTERFACE0;
	else if (port < 32)
		return CVMX_PKO_QUEUES_PER_PORT_INTERFACE1;
	else if (port < 36)
		return CVMX_PKO_QUEUES_PER_PORT_PCI;
	else if (port < 40)
		return CVMX_PKO_QUEUES_PER_PORT_LOOP;
	else
		return 0;
}

static inline void cvmx_pko_get_port_status(uint64_t port_num, uint64_t clear,
					    cvmx_pko_port_status_t *status)
{
	union cvmx_pko_reg_read_idx pko_reg_read_idx;
	union cvmx_pko_mem_count0 pko_mem_count0;
	union cvmx_pko_mem_count1 pko_mem_count1;

	pko_reg_read_idx.u64 = 0;
	pko_reg_read_idx.s.index = port_num;
	cvmx_write_csr(CVMX_PKO_REG_READ_IDX, pko_reg_read_idx.u64);

	pko_mem_count0.u64 = cvmx_read_csr(CVMX_PKO_MEM_COUNT0);
	status->packets = pko_mem_count0.s.count;
	if (clear) {
		pko_mem_count0.s.count = port_num;
		cvmx_write_csr(CVMX_PKO_MEM_COUNT0, pko_mem_count0.u64);
	}

	pko_mem_count1.u64 = cvmx_read_csr(CVMX_PKO_MEM_COUNT1);
	status->octets = pko_mem_count1.s.count;
	if (clear) {
		pko_mem_count1.s.count = port_num;
		cvmx_write_csr(CVMX_PKO_MEM_COUNT1, pko_mem_count1.u64);
	}

	if (OCTEON_IS_MODEL(OCTEON_CN3XXX)) {
		union cvmx_pko_mem_debug9 debug9;
		pko_reg_read_idx.s.index = cvmx_pko_get_base_queue(port_num);
		cvmx_write_csr(CVMX_PKO_REG_READ_IDX, pko_reg_read_idx.u64);
		debug9.u64 = cvmx_read_csr(CVMX_PKO_MEM_DEBUG9);
		status->doorbell = debug9.cn38xx.doorbell;
	} else {
		union cvmx_pko_mem_debug8 debug8;
		pko_reg_read_idx.s.index = cvmx_pko_get_base_queue(port_num);
		cvmx_write_csr(CVMX_PKO_REG_READ_IDX, pko_reg_read_idx.u64);
		debug8.u64 = cvmx_read_csr(CVMX_PKO_MEM_DEBUG8);
		status->doorbell = debug8.cn58xx.doorbell;
	}
}

extern int cvmx_pko_rate_limit_packets(int port, int packets_s, int burst);

extern int cvmx_pko_rate_limit_bits(int port, uint64_t bits_s, int burst);

#endif /* __CVMX_PKO_H__ */
