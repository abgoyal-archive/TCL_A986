

#ifndef __sctp_ulpqueue_h__
#define __sctp_ulpqueue_h__

/* A structure to carry information to the ULP (e.g. Sockets API) */
struct sctp_ulpq {
	char malloced;
	char pd_mode;
	struct sctp_association *asoc;
	struct sk_buff_head reasm;
	struct sk_buff_head lobby;
};

/* Prototypes. */
struct sctp_ulpq *sctp_ulpq_init(struct sctp_ulpq *,
				 struct sctp_association *);
void sctp_ulpq_flush(struct sctp_ulpq *ulpq);
void sctp_ulpq_free(struct sctp_ulpq *);

/* Add a new DATA chunk for processing. */
int sctp_ulpq_tail_data(struct sctp_ulpq *, struct sctp_chunk *, gfp_t);

/* Add a new event for propagation to the ULP. */
int sctp_ulpq_tail_event(struct sctp_ulpq *, struct sctp_ulpevent *ev);

/* Renege previously received chunks.  */
void sctp_ulpq_renege(struct sctp_ulpq *, struct sctp_chunk *, gfp_t);

/* Perform partial delivery. */
void sctp_ulpq_partial_delivery(struct sctp_ulpq *, struct sctp_chunk *, gfp_t);

/* Abort the partial delivery. */
void sctp_ulpq_abort_pd(struct sctp_ulpq *, gfp_t);

/* Clear the partial data delivery condition on this socket. */
int sctp_clear_pd(struct sock *sk, struct sctp_association *asoc);

/* Skip over an SSN. */
void sctp_ulpq_skip(struct sctp_ulpq *ulpq, __u16 sid, __u16 ssn);

void sctp_ulpq_reasm_flushtsn(struct sctp_ulpq *, __u32);
#endif /* __sctp_ulpqueue_h__ */







