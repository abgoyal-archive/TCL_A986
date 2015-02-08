

#include <asm/system.h>
#include <linux/gfp.h>
#include <linux/sched.h>
#include <linux/types.h>

#include "dt3155.h"
#include "dt3155_drv.h"
#include "dt3155_io.h"
#include "dt3155_isr.h"
#include "allocator.h"

#define FOUR_MB         (0x0400000)  /* Can't DMA accross a 4MB boundary!*/
#define UPPER_10_BITS   (0x3FF<<22)  /* Can't DMA accross a 4MB boundary!*/


/* Pointer into global structure for handling buffers */
struct dt3155_fbuffer *dt3155_fbuffer[MAXBOARDS] = {NULL
#if MAXBOARDS == 2
						      , NULL
#endif
};



bool are_empty_buffers(int m)
{
  return dt3155_fbuffer[m]->empty_len;
}

void push_empty(int index, int m)
{
  dt3155_fbuffer[m]->empty_buffers[dt3155_fbuffer[m]->empty_len] = index;
  dt3155_fbuffer[m]->empty_len++;
}

int pop_empty(int m)
{
  dt3155_fbuffer[m]->empty_len--;
  return dt3155_fbuffer[m]->empty_buffers[dt3155_fbuffer[m]->empty_len];
}

bool is_ready_buf_empty(int m)
{
  return ((dt3155_fbuffer[m]->ready_len) == 0);
}

bool is_ready_buf_full(int m)
{
  return dt3155_fbuffer[m]->ready_len == dt3155_fbuffer[m]->nbuffers;
}

void push_ready(int m, int index)
{
  int head = dt3155_fbuffer[m]->ready_head;

  dt3155_fbuffer[m]->ready_que[head] = index;
  dt3155_fbuffer[m]->ready_head = ((head + 1) %
				      (dt3155_fbuffer[m]->nbuffers));
  dt3155_fbuffer[m]->ready_len++;

}

static int get_tail(int m)
{
  return (dt3155_fbuffer[m]->ready_head -
	   dt3155_fbuffer[m]->ready_len +
	   dt3155_fbuffer[m]->nbuffers)%
	  (dt3155_fbuffer[m]->nbuffers);
}



int pop_ready(int m)
{
  int tail;
  tail = get_tail(m);
  dt3155_fbuffer[m]->ready_len--;
  return dt3155_fbuffer[m]->ready_que[tail];
}


void printques(int m)
{
  int head = dt3155_fbuffer[m]->ready_head;
  int tail;
  int num = dt3155_fbuffer[m]->nbuffers;
  int frame_index;
  int index;

  tail = get_tail(m);

  printk("\n R:");
    for (index = tail; index != head; index++, index = index % (num)) {
	frame_index = dt3155_fbuffer[m]->ready_que[index];
	printk(" %d ", frame_index);
    }

  printk("\n E:");
    for (index = 0; index < dt3155_fbuffer[m]->empty_len; index++) {
	frame_index = dt3155_fbuffer[m]->empty_buffers[index];
	printk(" %d ", frame_index);
    }

  frame_index = dt3155_fbuffer[m]->active_buf;
  printk("\n A: %d", frame_index);

  frame_index = dt3155_fbuffer[m]->locked_buf;
  printk("\n L: %d\n", frame_index);

}

u32 adjust_4MB(u32 buf_addr, u32 bufsize)
{
    if (((buf_addr+bufsize) & UPPER_10_BITS) != (buf_addr & UPPER_10_BITS))
	return (buf_addr+bufsize) & UPPER_10_BITS;
    else
	return buf_addr;
}


void allocate_buffers(u32 *buf_addr, u32* total_size_kbs,
		       u32 bufsize)
{
  /* Compute the minimum amount of memory guaranteed to hold all
     MAXBUFFERS such that no buffer crosses the 4MB boundary.
     Store this value in the variable "full_size" */

  u32 allocator_max;
  u32 bufs_per_chunk = (FOUR_MB / bufsize);
  u32 filled_chunks = (MAXBUFFERS-1) / bufs_per_chunk;
  u32 leftover_bufs = MAXBUFFERS - filled_chunks * bufs_per_chunk;

  u32 full_size = bufsize      /* possibly unusable part of 1st chunk */
    + filled_chunks * FOUR_MB   /* max # of completely filled 4mb chunks */
    + leftover_bufs * bufsize;  /* these buffs will be in a partly filled
				   chunk at beginning or end */

  u32 full_size_kbs = 1 + (full_size-1) / 1024;
  u32 min_size_kbs = 2*ndevices*bufsize / 1024;
  u32 size_kbs;

  /* Now, try to allocate full_size.  If this fails, keep trying for
     less & less memory until it succeeds. */
#ifndef STANDALONE_ALLOCATOR
  /* initialize the allocator            */
  allocator_init(&allocator_max);
#endif
  size_kbs = full_size_kbs;
  *buf_addr = 0;
  printk("DT3155: We would like to get: %d KB\n", full_size_kbs);
  printk("DT3155: ...but need at least: %d KB\n", min_size_kbs);
  printk("DT3155: ...the allocator has: %d KB\n", allocator_max);
  size_kbs = (full_size_kbs <= allocator_max ? full_size_kbs : allocator_max);
    if (size_kbs > min_size_kbs) {
	if ((*buf_addr = allocator_allocate_dma(size_kbs, GFP_KERNEL)) != 0) {
		printk("DT3155:  Managed to allocate: %d KB\n", size_kbs);
		*total_size_kbs = size_kbs;
		return;
	}
    }
  /* If we got here, the allocation failed */
  printk("DT3155: Allocator failed!\n");
  *buf_addr = 0;
  *total_size_kbs = 0;
  return;

}


u32 dt3155_setup_buffers(u32 *allocatorAddr)

{
  u32 index;
  u32 rambuff_addr; /* start of allocation */
  u32 rambuff_size; /* total size allocated to driver */
  u32 rambuff_acm;  /* accumlator, keep track of how much
			  is left after being split up*/
  u32 rambuff_end;  /* end of rambuff */
  u32 numbufs;      /* number of useful buffers allocated (per device) */
  u32 bufsize      = DT3155_MAX_ROWS * DT3155_MAX_COLS;
  int m;               /* minor # of device, looped for all devs */

  /* zero the fbuffer status and address structure */
    for (m = 0; m < ndevices; m++) {
	dt3155_fbuffer[m] = &(dt3155_status[m].fbuffer);

      /* Make sure the buffering variables are consistent */
      {
	u8 *ptr = (u8 *) dt3155_fbuffer[m];
		for (index = 0; index < sizeof(struct dt3155_fbuffer); index++)
			*(ptr++) = 0;
      }
    }

  /* allocate a large contiguous chunk of RAM */
  allocate_buffers(&rambuff_addr, &rambuff_size, bufsize);
  printk("DT3155: mem info\n");
  printk("  - rambuf_addr = 0x%x\n", rambuff_addr);
  printk("  - length (kb) = %u\n", rambuff_size);
    if (rambuff_addr == 0) {
	printk(KERN_INFO
	    "DT3155: Error setup_buffers() allocator dma failed\n");
	return -ENOMEM;
    }
  *allocatorAddr = rambuff_addr;
  rambuff_end = rambuff_addr + 1024 * rambuff_size;

  /* after allocation, we need to count how many useful buffers there
     are so we can give an equal number to each device */
  rambuff_acm = rambuff_addr;
    for (index = 0; index < MAXBUFFERS; index++) {
	rambuff_acm = adjust_4MB(rambuff_acm, bufsize);/*avoid spanning 4MB bdry*/
	if (rambuff_acm + bufsize > rambuff_end)
		break;
	rambuff_acm += bufsize;
    }
  /* Following line is OK, will waste buffers if index
   * not evenly divisible by ndevices -NJC*/
  numbufs = index / ndevices;
  printk("  - numbufs = %u\n", numbufs);
    if (numbufs < 2) {
	printk(KERN_INFO
	"DT3155: Error setup_buffers() couldn't allocate 2 bufs/board\n");
	return -ENOMEM;
    }

  /* now that we have board memory we spit it up */
  /* between the boards and the buffers          */
    rambuff_acm = rambuff_addr;
    for (m = 0; m < ndevices; m++) {
	rambuff_acm = adjust_4MB(rambuff_acm, bufsize);

	/* Save the start of this boards buffer space (for mmap).  */
	dt3155_status[m].mem_addr = rambuff_acm;

	for (index = 0; index < numbufs; index++) {
		rambuff_acm = adjust_4MB(rambuff_acm, bufsize);
		if (rambuff_acm + bufsize > rambuff_end) {
			/* Should never happen */
			printk("DT3155 PROGRAM ERROR (GCS)\n"
			"Error distributing allocated buffers\n");
			return -ENOMEM;
		}

		dt3155_fbuffer[m]->frame_info[index].addr = rambuff_acm;
		push_empty(index, m);
		/* printk("  - Buffer : %lx\n",
		* dt3155_fbuffer[m]->frame_info[index].addr);
		*/
		dt3155_fbuffer[m]->nbuffers += 1;
		rambuff_acm += bufsize;
	}

	/* Make sure there is an active buffer there. */
	dt3155_fbuffer[m]->active_buf    = pop_empty(m);
	dt3155_fbuffer[m]->even_happened = 0;
	dt3155_fbuffer[m]->even_stopped  = 0;

	/* make sure there is no locked_buf JML 2/28/00 */
	dt3155_fbuffer[m]->locked_buf = -1;

	dt3155_status[m].mem_size =
	rambuff_acm - dt3155_status[m].mem_addr;

	/* setup the ready queue */
	dt3155_fbuffer[m]->ready_head = 0;
	dt3155_fbuffer[m]->ready_len = 0;
	printk("Available buffers for device %d: %d\n",
	    m, dt3155_fbuffer[m]->nbuffers);
    }

    return 1;
}

static void internal_release_locked_buffer(int m)
{
  /* Pointer into global structure for handling buffers */
    if (dt3155_fbuffer[m]->locked_buf >= 0) {
	push_empty(dt3155_fbuffer[m]->locked_buf, m);
	dt3155_fbuffer[m]->locked_buf = -1;
    }
}


void dt3155_release_locked_buffer(int m)
{
	unsigned long int flags;
	local_save_flags(flags);
	local_irq_disable();
	internal_release_locked_buffer(m);
	local_irq_restore(flags);
}


int dt3155_flush(int m)
{
  int index;
  unsigned long int flags;
  local_save_flags(flags);
  local_irq_disable();

  internal_release_locked_buffer(m);
  dt3155_fbuffer[m]->empty_len = 0;

    for (index = 0; index < dt3155_fbuffer[m]->nbuffers; index++)
	push_empty(index,  m);

  /* Make sure there is an active buffer there. */
  dt3155_fbuffer[m]->active_buf = pop_empty(m);

  dt3155_fbuffer[m]->even_happened = 0;
  dt3155_fbuffer[m]->even_stopped  = 0;

  /* setup the ready queue  */
  dt3155_fbuffer[m]->ready_head = 0;
  dt3155_fbuffer[m]->ready_len = 0;

  local_irq_restore(flags);

  return 0;
}

int dt3155_get_ready_buffer(int m)
{
  int frame_index;
  unsigned long int flags;
  local_save_flags(flags);
  local_irq_disable();

#ifdef DEBUG_QUES_A
  printques(m);
#endif

  internal_release_locked_buffer(m);

    if (is_ready_buf_empty(m))
	frame_index = -1;
    else {
	frame_index = pop_ready(m);
	dt3155_fbuffer[m]->locked_buf = frame_index;
    }

#ifdef DEBUG_QUES_B
  printques(m);
#endif

  local_irq_restore(flags);

  return frame_index;
}
