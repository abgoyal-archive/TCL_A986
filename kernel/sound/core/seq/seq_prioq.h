
#ifndef __SND_SEQ_PRIOQ_H
#define __SND_SEQ_PRIOQ_H

#include "seq_memory.h"


/* === PRIOQ === */

struct snd_seq_prioq {
	struct snd_seq_event_cell *head;      /* pointer to head of prioq */
	struct snd_seq_event_cell *tail;      /* pointer to tail of prioq */
	int cells;
	spinlock_t lock;
};


/* create new prioq (constructor) */
struct snd_seq_prioq *snd_seq_prioq_new(void);

/* delete prioq (destructor) */
void snd_seq_prioq_delete(struct snd_seq_prioq **fifo);

/* enqueue cell to prioq */
int snd_seq_prioq_cell_in(struct snd_seq_prioq *f, struct snd_seq_event_cell *cell);

/* dequeue cell from prioq */ 
struct snd_seq_event_cell *snd_seq_prioq_cell_out(struct snd_seq_prioq *f);

/* return number of events available in prioq */
int snd_seq_prioq_avail(struct snd_seq_prioq *f);

/* peek at cell at the head of the prioq */
struct snd_seq_event_cell *snd_seq_prioq_cell_peek(struct snd_seq_prioq *f);

/* client left queue */
void snd_seq_prioq_leave(struct snd_seq_prioq *f, int client, int timestamp);        

/* Remove events */
void snd_seq_prioq_remove_events(struct snd_seq_prioq *f, int client,
				 struct snd_seq_remove_events *info);

#endif
