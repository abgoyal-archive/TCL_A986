

#ifndef _NET_TIPC_H_
#define _NET_TIPC_H_

#ifdef __KERNEL__

#include <linux/tipc.h>
#include <linux/skbuff.h>



u32 tipc_get_addr(void);

#define TIPC_NOT_RUNNING  0
#define TIPC_NODE_MODE    1
#define TIPC_NET_MODE     2

typedef void (*tipc_mode_event)(void *usr_handle, int mode, u32 addr);

int tipc_attach(unsigned int *userref, tipc_mode_event, void *usr_handle);

void tipc_detach(unsigned int userref);

int tipc_get_mode(void);


typedef void (*tipc_msg_err_event) (void *usr_handle,
				    u32 portref,
				    struct sk_buff **buf,
				    unsigned char const *data,
				    unsigned int size,
				    int reason, 
				    struct tipc_portid const *attmpt_destid);

typedef void (*tipc_named_msg_err_event) (void *usr_handle,
					  u32 portref,
					  struct sk_buff **buf,
					  unsigned char const *data,
					  unsigned int size,
					  int reason, 
					  struct tipc_name_seq const *attmpt_dest);

typedef void (*tipc_conn_shutdown_event) (void *usr_handle,
					  u32 portref,
					  struct sk_buff **buf,
					  unsigned char const *data,
					  unsigned int size,
					  int reason);

typedef void (*tipc_msg_event) (void *usr_handle,
				u32 portref,
				struct sk_buff **buf,
				unsigned char const *data,
				unsigned int size,
				unsigned int importance, 
				struct tipc_portid const *origin);

typedef void (*tipc_named_msg_event) (void *usr_handle,
				      u32 portref,
				      struct sk_buff **buf,
				      unsigned char const *data,
				      unsigned int size,
				      unsigned int importance, 
				      struct tipc_portid const *orig,
				      struct tipc_name_seq const *dest);

typedef void (*tipc_conn_msg_event) (void *usr_handle,
				     u32 portref,
				     struct sk_buff **buf,
				     unsigned char const *data,
				     unsigned int size);

typedef void (*tipc_continue_event) (void *usr_handle, 
				     u32 portref);

int tipc_createport(unsigned int tipc_user, 
		    void *usr_handle, 
		    unsigned int importance, 
		    tipc_msg_err_event error_cb, 
		    tipc_named_msg_err_event named_error_cb, 
		    tipc_conn_shutdown_event conn_error_cb, 
		    tipc_msg_event message_cb, 
		    tipc_named_msg_event named_message_cb, 
		    tipc_conn_msg_event conn_message_cb, 
		    tipc_continue_event continue_event_cb,
		    u32 *portref);

int tipc_deleteport(u32 portref);

int tipc_ownidentity(u32 portref, struct tipc_portid *port);

int tipc_portimportance(u32 portref, unsigned int *importance);
int tipc_set_portimportance(u32 portref, unsigned int importance);

int tipc_portunreliable(u32 portref, unsigned int *isunreliable);
int tipc_set_portunreliable(u32 portref, unsigned int isunreliable);

int tipc_portunreturnable(u32 portref, unsigned int *isunreturnable);
int tipc_set_portunreturnable(u32 portref, unsigned int isunreturnable);

int tipc_publish(u32 portref, unsigned int scope, 
		 struct tipc_name_seq const *name_seq);
int tipc_withdraw(u32 portref, unsigned int scope,
		  struct tipc_name_seq const *name_seq);

int tipc_connect2port(u32 portref, struct tipc_portid const *port);

int tipc_disconnect(u32 portref);

int tipc_shutdown(u32 ref);

int tipc_isconnected(u32 portref, int *isconnected);

int tipc_peer(u32 portref, struct tipc_portid *peer);

int tipc_ref_valid(u32 portref); 


#define TIPC_PORT_IMPORTANCE 100	/* send using current port setting */


int tipc_send(u32 portref,
	      unsigned int num_sect,
	      struct iovec const *msg_sect);

int tipc_send_buf(u32 portref,
		  struct sk_buff *buf,
		  unsigned int dsz);

int tipc_send2name(u32 portref, 
		   struct tipc_name const *name, 
		   u32 domain,
		   unsigned int num_sect,
		   struct iovec const *msg_sect);

int tipc_send_buf2name(u32 portref,
		       struct tipc_name const *name,
		       u32 domain,
		       struct sk_buff *buf,
		       unsigned int dsz);

int tipc_forward2name(u32 portref, 
		      struct tipc_name const *name, 
		      u32 domain,
		      unsigned int section_count,
		      struct iovec const *msg_sect,
		      struct tipc_portid const *origin,
		      unsigned int importance);

int tipc_forward_buf2name(u32 portref,
			  struct tipc_name const *name,
			  u32 domain,
			  struct sk_buff *buf,
			  unsigned int dsz,
			  struct tipc_portid const *orig,
			  unsigned int importance);

int tipc_send2port(u32 portref,
		   struct tipc_portid const *dest,
		   unsigned int num_sect,
		   struct iovec const *msg_sect);

int tipc_send_buf2port(u32 portref,
		       struct tipc_portid const *dest,
		       struct sk_buff *buf,
		       unsigned int dsz);

int tipc_forward2port(u32 portref,
		      struct tipc_portid const *dest,
		      unsigned int num_sect,
		      struct iovec const *msg_sect,
		      struct tipc_portid const *origin,
		      unsigned int importance);

int tipc_forward_buf2port(u32 portref,
			  struct tipc_portid const *dest,
			  struct sk_buff *buf,
			  unsigned int dsz,
			  struct tipc_portid const *orig,
			  unsigned int importance);

int tipc_multicast(u32 portref, 
		   struct tipc_name_seq const *seq, 
		   u32 domain,	/* currently unused */
		   unsigned int section_count,
		   struct iovec const *msg);

#if 0
int tipc_multicast_buf(u32 portref, 
		       struct tipc_name_seq const *seq, 
		       u32 domain,
		       void *buf,
		       unsigned int size);
#endif


int tipc_ispublished(struct tipc_name const *name);


unsigned int tipc_available_nodes(const u32 domain);

#endif

#endif
