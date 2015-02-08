

#ifndef OPROFILE_BUFFER_SYNC_H
#define OPROFILE_BUFFER_SYNC_H

/* add the necessary profiling hooks */
int sync_start(void);

/* remove the hooks */
void sync_stop(void);

/* sync the given CPU's buffer */
void sync_buffer(int cpu);

#endif /* OPROFILE_BUFFER_SYNC_H */
