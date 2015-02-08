
#ifndef MSGQUEUE_H
#define MSGQUEUE_H

struct message {
    char msg[8];
    int length;
    int fifo;
};

struct msgqueue_entry {
    struct message msg;
    struct msgqueue_entry *next;
};

#define NR_MESSAGES 4

typedef struct {
    struct msgqueue_entry *qe;
    struct msgqueue_entry *free;
    struct msgqueue_entry entries[NR_MESSAGES];
} MsgQueue_t;

extern void msgqueue_initialise(MsgQueue_t *msgq);

extern void msgqueue_free(MsgQueue_t *msgq);

extern int msgqueue_msglength(MsgQueue_t *msgq);

extern struct message *msgqueue_getmsg(MsgQueue_t *msgq, int msgno);

extern int msgqueue_addmsg(MsgQueue_t *msgq, int length, ...);

extern void msgqueue_flush(MsgQueue_t *msgq);

#endif
