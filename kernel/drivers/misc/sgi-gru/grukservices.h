

#ifndef __GRU_KSERVICES_H_
#define __GRU_KSERVICES_H_



struct gru_message_queue_desc {
	void		*mq;			/* message queue vaddress */
	unsigned long	mq_gpa;			/* global address of mq */
	int		qlines;			/* queue size in CL */
	int		interrupt_vector;	/* interrupt vector */
	int		interrupt_pnode;	/* pnode for interrupt */
	int		interrupt_apicid;	/* lapicid for interrupt */
};

extern int gru_create_message_queue(struct gru_message_queue_desc *mqd,
		void *p, unsigned int bytes, int nasid, int vector, int apicid);

extern int gru_send_message_gpa(struct gru_message_queue_desc *mqd,
			void *mesg, unsigned int bytes);

/* Status values for gru_send_message() */
#define MQE_OK			0	/* message sent successfully */
#define MQE_CONGESTION		1	/* temporary congestion, try again */
#define MQE_QUEUE_FULL		2	/* queue is full */
#define MQE_UNEXPECTED_CB_ERR	3	/* unexpected CB error */
#define MQE_PAGE_OVERFLOW	10	/* BUG - queue overflowed a page */
#define MQE_BUG_NO_RESOURCES	11	/* BUG - could not alloc GRU cb/dsr */

extern void gru_free_message(struct gru_message_queue_desc *mqd,
			     void *mesq);

extern void *gru_get_next_message(struct gru_message_queue_desc *mqd);


int gru_read_gpa(unsigned long *value, unsigned long gpa);


extern int gru_copy_gpa(unsigned long dest_gpa, unsigned long src_gpa,
							unsigned int bytes);

extern unsigned long gru_reserve_async_resources(int blade_id, int cbrs, int dsr_bytes,
				struct completion *cmp);

extern void gru_release_async_resources(unsigned long han);

extern void gru_wait_async_cbr(unsigned long han);

extern void gru_lock_async_resource(unsigned long han,  void **cb, void **dsr);

extern void gru_unlock_async_resource(unsigned long han);

#endif 		/* __GRU_KSERVICES_H_ */
