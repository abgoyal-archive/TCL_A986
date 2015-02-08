


#ifndef __CVMX_CMD_QUEUE_H__
#define __CVMX_CMD_QUEUE_H__

#include <linux/prefetch.h>

#include "cvmx-fpa.h"
#ifndef CVMX_CMD_QUEUE_ENABLE_MAX_DEPTH
#define CVMX_CMD_QUEUE_ENABLE_MAX_DEPTH 0
#endif

typedef enum {
	CVMX_CMD_QUEUE_PKO_BASE = 0x00000,

#define CVMX_CMD_QUEUE_PKO(queue) \
	((cvmx_cmd_queue_id_t)(CVMX_CMD_QUEUE_PKO_BASE + (0xffff&(queue))))

	CVMX_CMD_QUEUE_ZIP = 0x10000,
	CVMX_CMD_QUEUE_DFA = 0x20000,
	CVMX_CMD_QUEUE_RAID = 0x30000,
	CVMX_CMD_QUEUE_DMA_BASE = 0x40000,

#define CVMX_CMD_QUEUE_DMA(queue) \
	((cvmx_cmd_queue_id_t)(CVMX_CMD_QUEUE_DMA_BASE + (0xffff&(queue))))

	CVMX_CMD_QUEUE_END = 0x50000,
} cvmx_cmd_queue_id_t;

typedef enum {
	CVMX_CMD_QUEUE_SUCCESS = 0,
	CVMX_CMD_QUEUE_NO_MEMORY = -1,
	CVMX_CMD_QUEUE_FULL = -2,
	CVMX_CMD_QUEUE_INVALID_PARAM = -3,
	CVMX_CMD_QUEUE_ALREADY_SETUP = -4,
} cvmx_cmd_queue_result_t;

typedef struct {
	/* You have lock when this is your ticket */
	uint8_t now_serving;
	uint64_t unused1:24;
	/* Maximum outstanding command words */
	uint32_t max_depth;
	/* FPA pool buffers come from */
	uint64_t fpa_pool:3;
	/* Top of command buffer pointer shifted 7 */
	uint64_t base_ptr_div128:29;
	uint64_t unused2:6;
	/* FPA buffer size in 64bit words minus 1 */
	uint64_t pool_size_m1:13;
	/* Number of comamnds already used in buffer */
	uint64_t index:13;
} __cvmx_cmd_queue_state_t;

typedef struct {
	uint64_t ticket[(CVMX_CMD_QUEUE_END >> 16) * 256];
	__cvmx_cmd_queue_state_t state[(CVMX_CMD_QUEUE_END >> 16) * 256];
} __cvmx_cmd_queue_all_state_t;

cvmx_cmd_queue_result_t cvmx_cmd_queue_initialize(cvmx_cmd_queue_id_t queue_id,
						  int max_depth, int fpa_pool,
						  int pool_size);

cvmx_cmd_queue_result_t cvmx_cmd_queue_shutdown(cvmx_cmd_queue_id_t queue_id);

int cvmx_cmd_queue_length(cvmx_cmd_queue_id_t queue_id);

void *cvmx_cmd_queue_buffer(cvmx_cmd_queue_id_t queue_id);

static inline int __cvmx_cmd_queue_get_index(cvmx_cmd_queue_id_t queue_id)
{
	/*
	 * Warning: This code currently only works with devices that
	 * have 256 queues or less. Devices with more than 16 queues
	 * are layed out in memory to allow cores quick access to
	 * every 16th queue. This reduces cache thrashing when you are
	 * running 16 queues per port to support lockless operation.
	 */
	int unit = queue_id >> 16;
	int q = (queue_id >> 4) & 0xf;
	int core = queue_id & 0xf;
	return unit * 256 + core * 16 + q;
}

static inline void __cvmx_cmd_queue_lock(cvmx_cmd_queue_id_t queue_id,
					 __cvmx_cmd_queue_state_t *qptr)
{
	extern __cvmx_cmd_queue_all_state_t
	    *__cvmx_cmd_queue_state_ptr;
	int tmp;
	int my_ticket;
	prefetch(qptr);
	asm volatile (
		".set push\n"
		".set noreorder\n"
		"1:\n"
		/* Atomic add one to ticket_ptr */
		"ll     %[my_ticket], %[ticket_ptr]\n"
		/* and store the original value */
		"li     %[ticket], 1\n"
		/* in my_ticket */
		"baddu  %[ticket], %[my_ticket]\n"
		"sc     %[ticket], %[ticket_ptr]\n"
		"beqz   %[ticket], 1b\n"
		" nop\n"
		/* Load the current now_serving ticket */
		"lbu    %[ticket], %[now_serving]\n"
		"2:\n"
		/* Jump out if now_serving == my_ticket */
		"beq    %[ticket], %[my_ticket], 4f\n"
		/* Find out how many tickets are in front of me */
		" subu   %[ticket], %[my_ticket], %[ticket]\n"
		/* Use tickets in front of me minus one to delay */
		"subu  %[ticket], 1\n"
		/* Delay will be ((tickets in front)-1)*32 loops */
		"cins   %[ticket], %[ticket], 5, 7\n"
		"3:\n"
		/* Loop here until our ticket might be up */
		"bnez   %[ticket], 3b\n"
		" subu  %[ticket], 1\n"
		/* Jump back up to check out ticket again */
		"b      2b\n"
		/* Load the current now_serving ticket */
		" lbu   %[ticket], %[now_serving]\n"
		"4:\n"
		".set pop\n" :
		[ticket_ptr] "=m"(__cvmx_cmd_queue_state_ptr->ticket[__cvmx_cmd_queue_get_index(queue_id)]),
		[now_serving] "=m"(qptr->now_serving), [ticket] "=r"(tmp),
		[my_ticket] "=r"(my_ticket)
	    );
}

static inline void __cvmx_cmd_queue_unlock(__cvmx_cmd_queue_state_t *qptr)
{
	qptr->now_serving++;
	CVMX_SYNCWS;
}

static inline __cvmx_cmd_queue_state_t
    *__cvmx_cmd_queue_get_state(cvmx_cmd_queue_id_t queue_id)
{
	extern __cvmx_cmd_queue_all_state_t
	    *__cvmx_cmd_queue_state_ptr;
	return &__cvmx_cmd_queue_state_ptr->
	    state[__cvmx_cmd_queue_get_index(queue_id)];
}

static inline cvmx_cmd_queue_result_t cvmx_cmd_queue_write(cvmx_cmd_queue_id_t
							   queue_id,
							   int use_locking,
							   int cmd_count,
							   uint64_t *cmds)
{
	__cvmx_cmd_queue_state_t *qptr = __cvmx_cmd_queue_get_state(queue_id);

	/* Make sure nobody else is updating the same queue */
	if (likely(use_locking))
		__cvmx_cmd_queue_lock(queue_id, qptr);

	/*
	 * If a max queue length was specified then make sure we don't
	 * exceed it. If any part of the command would be below the
	 * limit we allow it.
	 */
	if (CVMX_CMD_QUEUE_ENABLE_MAX_DEPTH && unlikely(qptr->max_depth)) {
		if (unlikely
		    (cvmx_cmd_queue_length(queue_id) > (int)qptr->max_depth)) {
			if (likely(use_locking))
				__cvmx_cmd_queue_unlock(qptr);
			return CVMX_CMD_QUEUE_FULL;
		}
	}

	/*
	 * Normally there is plenty of room in the current buffer for
	 * the command.
	 */
	if (likely(qptr->index + cmd_count < qptr->pool_size_m1)) {
		uint64_t *ptr =
		    (uint64_t *) cvmx_phys_to_ptr((uint64_t) qptr->
						  base_ptr_div128 << 7);
		ptr += qptr->index;
		qptr->index += cmd_count;
		while (cmd_count--)
			*ptr++ = *cmds++;
	} else {
		uint64_t *ptr;
		int count;
		/*
		 * We need a new comamnd buffer. Fail if there isn't
		 * one available.
		 */
		uint64_t *new_buffer =
		    (uint64_t *) cvmx_fpa_alloc(qptr->fpa_pool);
		if (unlikely(new_buffer == NULL)) {
			if (likely(use_locking))
				__cvmx_cmd_queue_unlock(qptr);
			return CVMX_CMD_QUEUE_NO_MEMORY;
		}
		ptr =
		    (uint64_t *) cvmx_phys_to_ptr((uint64_t) qptr->
						  base_ptr_div128 << 7);
		/*
		 * Figure out how many command words will fit in this
		 * buffer. One location will be needed for the next
		 * buffer pointer.
		 */
		count = qptr->pool_size_m1 - qptr->index;
		ptr += qptr->index;
		cmd_count -= count;
		while (count--)
			*ptr++ = *cmds++;
		*ptr = cvmx_ptr_to_phys(new_buffer);
		/*
		 * The current buffer is full and has a link to the
		 * next buffer. Time to write the rest of the commands
		 * into the new buffer.
		 */
		qptr->base_ptr_div128 = *ptr >> 7;
		qptr->index = cmd_count;
		ptr = new_buffer;
		while (cmd_count--)
			*ptr++ = *cmds++;
	}

	/* All updates are complete. Release the lock and return */
	if (likely(use_locking))
		__cvmx_cmd_queue_unlock(qptr);
	return CVMX_CMD_QUEUE_SUCCESS;
}

static inline cvmx_cmd_queue_result_t cvmx_cmd_queue_write2(cvmx_cmd_queue_id_t
							    queue_id,
							    int use_locking,
							    uint64_t cmd1,
							    uint64_t cmd2)
{
	__cvmx_cmd_queue_state_t *qptr = __cvmx_cmd_queue_get_state(queue_id);

	/* Make sure nobody else is updating the same queue */
	if (likely(use_locking))
		__cvmx_cmd_queue_lock(queue_id, qptr);

	/*
	 * If a max queue length was specified then make sure we don't
	 * exceed it. If any part of the command would be below the
	 * limit we allow it.
	 */
	if (CVMX_CMD_QUEUE_ENABLE_MAX_DEPTH && unlikely(qptr->max_depth)) {
		if (unlikely
		    (cvmx_cmd_queue_length(queue_id) > (int)qptr->max_depth)) {
			if (likely(use_locking))
				__cvmx_cmd_queue_unlock(qptr);
			return CVMX_CMD_QUEUE_FULL;
		}
	}

	/*
	 * Normally there is plenty of room in the current buffer for
	 * the command.
	 */
	if (likely(qptr->index + 2 < qptr->pool_size_m1)) {
		uint64_t *ptr =
		    (uint64_t *) cvmx_phys_to_ptr((uint64_t) qptr->
						  base_ptr_div128 << 7);
		ptr += qptr->index;
		qptr->index += 2;
		ptr[0] = cmd1;
		ptr[1] = cmd2;
	} else {
		uint64_t *ptr;
		/*
		 * Figure out how many command words will fit in this
		 * buffer. One location will be needed for the next
		 * buffer pointer.
		 */
		int count = qptr->pool_size_m1 - qptr->index;
		/*
		 * We need a new comamnd buffer. Fail if there isn't
		 * one available.
		 */
		uint64_t *new_buffer =
		    (uint64_t *) cvmx_fpa_alloc(qptr->fpa_pool);
		if (unlikely(new_buffer == NULL)) {
			if (likely(use_locking))
				__cvmx_cmd_queue_unlock(qptr);
			return CVMX_CMD_QUEUE_NO_MEMORY;
		}
		count--;
		ptr =
		    (uint64_t *) cvmx_phys_to_ptr((uint64_t) qptr->
						  base_ptr_div128 << 7);
		ptr += qptr->index;
		*ptr++ = cmd1;
		if (likely(count))
			*ptr++ = cmd2;
		*ptr = cvmx_ptr_to_phys(new_buffer);
		/*
		 * The current buffer is full and has a link to the
		 * next buffer. Time to write the rest of the commands
		 * into the new buffer.
		 */
		qptr->base_ptr_div128 = *ptr >> 7;
		qptr->index = 0;
		if (unlikely(count == 0)) {
			qptr->index = 1;
			new_buffer[0] = cmd2;
		}
	}

	/* All updates are complete. Release the lock and return */
	if (likely(use_locking))
		__cvmx_cmd_queue_unlock(qptr);
	return CVMX_CMD_QUEUE_SUCCESS;
}

static inline cvmx_cmd_queue_result_t cvmx_cmd_queue_write3(cvmx_cmd_queue_id_t
							    queue_id,
							    int use_locking,
							    uint64_t cmd1,
							    uint64_t cmd2,
							    uint64_t cmd3)
{
	__cvmx_cmd_queue_state_t *qptr = __cvmx_cmd_queue_get_state(queue_id);

	/* Make sure nobody else is updating the same queue */
	if (likely(use_locking))
		__cvmx_cmd_queue_lock(queue_id, qptr);

	/*
	 * If a max queue length was specified then make sure we don't
	 * exceed it. If any part of the command would be below the
	 * limit we allow it.
	 */
	if (CVMX_CMD_QUEUE_ENABLE_MAX_DEPTH && unlikely(qptr->max_depth)) {
		if (unlikely
		    (cvmx_cmd_queue_length(queue_id) > (int)qptr->max_depth)) {
			if (likely(use_locking))
				__cvmx_cmd_queue_unlock(qptr);
			return CVMX_CMD_QUEUE_FULL;
		}
	}

	/*
	 * Normally there is plenty of room in the current buffer for
	 * the command.
	 */
	if (likely(qptr->index + 3 < qptr->pool_size_m1)) {
		uint64_t *ptr =
		    (uint64_t *) cvmx_phys_to_ptr((uint64_t) qptr->
						  base_ptr_div128 << 7);
		ptr += qptr->index;
		qptr->index += 3;
		ptr[0] = cmd1;
		ptr[1] = cmd2;
		ptr[2] = cmd3;
	} else {
		uint64_t *ptr;
		/*
		 * Figure out how many command words will fit in this
		 * buffer. One location will be needed for the next
		 * buffer pointer
		 */
		int count = qptr->pool_size_m1 - qptr->index;
		/*
		 * We need a new comamnd buffer. Fail if there isn't
		 * one available
		 */
		uint64_t *new_buffer =
		    (uint64_t *) cvmx_fpa_alloc(qptr->fpa_pool);
		if (unlikely(new_buffer == NULL)) {
			if (likely(use_locking))
				__cvmx_cmd_queue_unlock(qptr);
			return CVMX_CMD_QUEUE_NO_MEMORY;
		}
		count--;
		ptr =
		    (uint64_t *) cvmx_phys_to_ptr((uint64_t) qptr->
						  base_ptr_div128 << 7);
		ptr += qptr->index;
		*ptr++ = cmd1;
		if (count) {
			*ptr++ = cmd2;
			if (count > 1)
				*ptr++ = cmd3;
		}
		*ptr = cvmx_ptr_to_phys(new_buffer);
		/*
		 * The current buffer is full and has a link to the
		 * next buffer. Time to write the rest of the commands
		 * into the new buffer.
		 */
		qptr->base_ptr_div128 = *ptr >> 7;
		qptr->index = 0;
		ptr = new_buffer;
		if (count == 0) {
			*ptr++ = cmd2;
			qptr->index++;
		}
		if (count < 2) {
			*ptr++ = cmd3;
			qptr->index++;
		}
	}

	/* All updates are complete. Release the lock and return */
	if (likely(use_locking))
		__cvmx_cmd_queue_unlock(qptr);
	return CVMX_CMD_QUEUE_SUCCESS;
}

#endif /* __CVMX_CMD_QUEUE_H__ */
