

#ifndef DT3155_ISR_H
#define DT3155_ISR_H

extern struct dt3155_fbuffer *dt3155_fbuffer[MAXBOARDS];

/* User functions for buffering */
/* Initialize the buffering system.  This should */
/* be called prior to enabling interrupts */

u32 dt3155_setup_buffers(u32 *allocatorAddr);

/* Get the next frame of data if it is ready.  Returns */
/* zero if no data is ready.  If there is data but */
/* the user has a locked buffer, it will unlock that */
/* buffer and return it to the free list. */

int dt3155_get_ready_buffer(int minor);

/* Return a locked buffer to the free list */

void dt3155_release_locked_buffer(int minor);

/* Flush the buffer system */
int dt3155_flush(int minor);


bool are_empty_buffers(int minor);
void push_empty(int index, int minor);

int  pop_empty(int minor);

bool is_ready_buf_empty(int minor);
bool is_ready_buf_full(int minor);

void push_ready(int minor, int index);
int  pop_ready(int minor);


#endif
