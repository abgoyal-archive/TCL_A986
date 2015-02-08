

#ifndef __LINUX_IPMI_H
#define __LINUX_IPMI_H

#include <linux/ipmi_msgdefs.h>
#include <linux/compiler.h>




#define IPMI_MAX_ADDR_SIZE 32
struct ipmi_addr {
	 /* Try to take these from the "Channel Medium Type" table
	    in section 6.5 of the IPMI 1.5 manual. */
	int   addr_type;
	short channel;
	char  data[IPMI_MAX_ADDR_SIZE];
};

#define IPMI_SYSTEM_INTERFACE_ADDR_TYPE	0x0c
struct ipmi_system_interface_addr {
	int           addr_type;
	short         channel;
	unsigned char lun;
};

/* An IPMB Address. */
#define IPMI_IPMB_ADDR_TYPE		0x01
#define IPMI_IPMB_BROADCAST_ADDR_TYPE	0x41
struct ipmi_ipmb_addr {
	int           addr_type;
	short         channel;
	unsigned char slave_addr;
	unsigned char lun;
};

#define IPMI_LAN_ADDR_TYPE		0x04
struct ipmi_lan_addr {
	int           addr_type;
	short         channel;
	unsigned char privilege;
	unsigned char session_handle;
	unsigned char remote_SWID;
	unsigned char local_SWID;
	unsigned char lun;
};


#define IPMI_BMC_CHANNEL  0xf
#define IPMI_NUM_CHANNELS 0x10

#define IPMI_CHAN_ALL     (~0)


struct ipmi_msg {
	unsigned char  netfn;
	unsigned char  cmd;
	unsigned short data_len;
	unsigned char  __user *data;
};

struct kernel_ipmi_msg {
	unsigned char  netfn;
	unsigned char  cmd;
	unsigned short data_len;
	unsigned char  *data;
};

#define IPMI_INVALID_CMD_COMPLETION_CODE	0xC1
#define IPMI_TIMEOUT_COMPLETION_CODE		0xC3
#define IPMI_UNKNOWN_ERR_COMPLETION_CODE	0xff


#define IPMI_RESPONSE_RECV_TYPE		1 /* A response to a command */
#define IPMI_ASYNC_EVENT_RECV_TYPE	2 /* Something from the event queue */
#define IPMI_CMD_RECV_TYPE		3 /* A command from somewhere else */
#define IPMI_RESPONSE_RESPONSE_TYPE	4 /* The response for
					      a sent response, giving any
					      error status for sending the
					      response.  When you send a
					      response message, this will
					      be returned. */
#define IPMI_OEM_RECV_TYPE		5 /* The response for OEM Channels */



#define IPMI_MAINTENANCE_MODE_AUTO	0
#define IPMI_MAINTENANCE_MODE_OFF	1
#define IPMI_MAINTENANCE_MODE_ON	2

#ifdef __KERNEL__

#include <linux/list.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/proc_fs.h>

typedef struct ipmi_user *ipmi_user_t;

struct ipmi_recv_msg {
	struct list_head link;

	/* The type of message as defined in the "Receive Types"
	   defines above. */
	int              recv_type;

	ipmi_user_t      user;
	struct ipmi_addr addr;
	long             msgid;
	struct kernel_ipmi_msg  msg;

	/* The user_msg_data is the data supplied when a message was
	   sent, if this is a response to a sent message.  If this is
	   not a response to a sent message, then user_msg_data will
	   be NULL.  If the user above is NULL, then this will be the
	   intf. */
	void             *user_msg_data;

	/* Call this when done with the message.  It will presumably free
	   the message and do any other necessary cleanup. */
	void (*done)(struct ipmi_recv_msg *msg);

	/* Place-holder for the data, don't make any assumptions about
	   the size or existance of this, since it may change. */
	unsigned char   msg_data[IPMI_MAX_MSG_LENGTH];
};

/* Allocate and free the receive message. */
void ipmi_free_recv_msg(struct ipmi_recv_msg *msg);

struct ipmi_user_hndl {
	/* Routine type to call when a message needs to be routed to
	   the upper layer.  This will be called with some locks held,
	   the only IPMI routines that can be called are ipmi_request
	   and the alloc/free operations.  The handler_data is the
	   variable supplied when the receive handler was registered. */
	void (*ipmi_recv_hndl)(struct ipmi_recv_msg *msg,
			       void                 *user_msg_data);

	/* Called when the interface detects a watchdog pre-timeout.  If
	   this is NULL, it will be ignored for the user. */
	void (*ipmi_watchdog_pretimeout)(void *handler_data);
};

/* Create a new user of the IPMI layer on the given interface number. */
int ipmi_create_user(unsigned int          if_num,
		     struct ipmi_user_hndl *handler,
		     void                  *handler_data,
		     ipmi_user_t           *user);

int ipmi_destroy_user(ipmi_user_t user);

/* Get the IPMI version of the BMC we are talking to. */
void ipmi_get_version(ipmi_user_t   user,
		      unsigned char *major,
		      unsigned char *minor);

int ipmi_set_my_address(ipmi_user_t   user,
			unsigned int  channel,
			unsigned char address);
int ipmi_get_my_address(ipmi_user_t   user,
			unsigned int  channel,
			unsigned char *address);
int ipmi_set_my_LUN(ipmi_user_t   user,
		    unsigned int  channel,
		    unsigned char LUN);
int ipmi_get_my_LUN(ipmi_user_t   user,
		    unsigned int  channel,
		    unsigned char *LUN);

int ipmi_request_settime(ipmi_user_t      user,
			 struct ipmi_addr *addr,
			 long             msgid,
			 struct kernel_ipmi_msg  *msg,
			 void             *user_msg_data,
			 int              priority,
			 int              max_retries,
			 unsigned int     retry_time_ms);

int ipmi_request_supply_msgs(ipmi_user_t          user,
			     struct ipmi_addr     *addr,
			     long                 msgid,
			     struct kernel_ipmi_msg *msg,
			     void                 *user_msg_data,
			     void                 *supplied_smi,
			     struct ipmi_recv_msg *supplied_recv,
			     int                  priority);

void ipmi_poll_interface(ipmi_user_t user);

int ipmi_register_for_cmd(ipmi_user_t   user,
			  unsigned char netfn,
			  unsigned char cmd,
			  unsigned int  chans);
int ipmi_unregister_for_cmd(ipmi_user_t   user,
			    unsigned char netfn,
			    unsigned char cmd,
			    unsigned int  chans);

int ipmi_get_maintenance_mode(ipmi_user_t user);
int ipmi_set_maintenance_mode(ipmi_user_t user, int mode);

int ipmi_set_gets_events(ipmi_user_t user, int val);

struct ipmi_smi_watcher {
	struct list_head link;

	/* You must set the owner to the current module, if you are in
	   a module (generally just set it to "THIS_MODULE"). */
	struct module *owner;

	/* These two are called with read locks held for the interface
	   the watcher list.  So you can add and remove users from the
	   IPMI interface, send messages, etc., but you cannot add
	   or remove SMI watchers or SMI interfaces. */
	void (*new_smi)(int if_num, struct device *dev);
	void (*smi_gone)(int if_num);
};

int ipmi_smi_watcher_register(struct ipmi_smi_watcher *watcher);
int ipmi_smi_watcher_unregister(struct ipmi_smi_watcher *watcher);


/* Return the maximum length of an IPMI address given it's type. */
unsigned int ipmi_addr_length(int addr_type);

/* Validate that the given IPMI address is valid. */
int ipmi_validate_addr(struct ipmi_addr *addr, int len);

#endif /* __KERNEL__ */





/* The magic IOCTL value for this interface. */
#define IPMI_IOC_MAGIC 'i'


/* Messages sent to the interface are this format. */
struct ipmi_req {
	unsigned char __user *addr; /* Address to send the message to. */
	unsigned int  addr_len;

	long    msgid; /* The sequence number for the message.  This
			  exact value will be reported back in the
			  response to this request if it is a command.
			  If it is a response, this will be used as
			  the sequence value for the response.  */

	struct ipmi_msg msg;
};
#define IPMICTL_SEND_COMMAND		_IOR(IPMI_IOC_MAGIC, 13,	\
					     struct ipmi_req)

struct ipmi_req_settime {
	struct ipmi_req req;

	/* See ipmi_request_settime() above for details on these
	   values. */
	int          retries;
	unsigned int retry_time_ms;
};
#define IPMICTL_SEND_COMMAND_SETTIME	_IOR(IPMI_IOC_MAGIC, 21,	\
					     struct ipmi_req_settime)

/* Messages received from the interface are this format. */
struct ipmi_recv {
	int     recv_type; /* Is this a command, response or an
			      asyncronous event. */

	unsigned char __user *addr;    /* Address the message was from is put
				   here.  The caller must supply the
				   memory. */
	unsigned int  addr_len; /* The size of the address buffer.
				   The caller supplies the full buffer
				   length, this value is updated to
				   the actual message length when the
				   message is received. */

	long    msgid; /* The sequence number specified in the request
			  if this is a response.  If this is a command,
			  this will be the sequence number from the
			  command. */

	struct ipmi_msg msg; /* The data field must point to a buffer.
				The data_size field must be set to the
				size of the message buffer.  The
				caller supplies the full buffer
				length, this value is updated to the
				actual message length when the message
				is received. */
};

#define IPMICTL_RECEIVE_MSG		_IOWR(IPMI_IOC_MAGIC, 12,	\
					      struct ipmi_recv)

#define IPMICTL_RECEIVE_MSG_TRUNC	_IOWR(IPMI_IOC_MAGIC, 11,	\
					      struct ipmi_recv)

/* Register to get commands from other entities on this interface. */
struct ipmi_cmdspec {
	unsigned char netfn;
	unsigned char cmd;
};

#define IPMICTL_REGISTER_FOR_CMD	_IOR(IPMI_IOC_MAGIC, 14,	\
					     struct ipmi_cmdspec)
#define IPMICTL_UNREGISTER_FOR_CMD	_IOR(IPMI_IOC_MAGIC, 15,	\
					     struct ipmi_cmdspec)

struct ipmi_cmdspec_chans {
	unsigned int netfn;
	unsigned int cmd;
	unsigned int chans;
};

#define IPMICTL_REGISTER_FOR_CMD_CHANS	_IOR(IPMI_IOC_MAGIC, 28,	\
					     struct ipmi_cmdspec_chans)
#define IPMICTL_UNREGISTER_FOR_CMD_CHANS _IOR(IPMI_IOC_MAGIC, 29,	\
					     struct ipmi_cmdspec_chans)

#define IPMICTL_SET_GETS_EVENTS_CMD	_IOR(IPMI_IOC_MAGIC, 16, int)

struct ipmi_channel_lun_address_set {
	unsigned short channel;
	unsigned char  value;
};
#define IPMICTL_SET_MY_CHANNEL_ADDRESS_CMD \
	_IOR(IPMI_IOC_MAGIC, 24, struct ipmi_channel_lun_address_set)
#define IPMICTL_GET_MY_CHANNEL_ADDRESS_CMD \
	_IOR(IPMI_IOC_MAGIC, 25, struct ipmi_channel_lun_address_set)
#define IPMICTL_SET_MY_CHANNEL_LUN_CMD \
	_IOR(IPMI_IOC_MAGIC, 26, struct ipmi_channel_lun_address_set)
#define IPMICTL_GET_MY_CHANNEL_LUN_CMD \
	_IOR(IPMI_IOC_MAGIC, 27, struct ipmi_channel_lun_address_set)
/* Legacy interfaces, these only set IPMB 0. */
#define IPMICTL_SET_MY_ADDRESS_CMD	_IOR(IPMI_IOC_MAGIC, 17, unsigned int)
#define IPMICTL_GET_MY_ADDRESS_CMD	_IOR(IPMI_IOC_MAGIC, 18, unsigned int)
#define IPMICTL_SET_MY_LUN_CMD		_IOR(IPMI_IOC_MAGIC, 19, unsigned int)
#define IPMICTL_GET_MY_LUN_CMD		_IOR(IPMI_IOC_MAGIC, 20, unsigned int)

struct ipmi_timing_parms {
	int          retries;
	unsigned int retry_time_ms;
};
#define IPMICTL_SET_TIMING_PARMS_CMD	_IOR(IPMI_IOC_MAGIC, 22, \
					     struct ipmi_timing_parms)
#define IPMICTL_GET_TIMING_PARMS_CMD	_IOR(IPMI_IOC_MAGIC, 23, \
					     struct ipmi_timing_parms)

#define IPMICTL_GET_MAINTENANCE_MODE_CMD	_IOR(IPMI_IOC_MAGIC, 30, int)
#define IPMICTL_SET_MAINTENANCE_MODE_CMD	_IOW(IPMI_IOC_MAGIC, 31, int)

#endif /* __LINUX_IPMI_H */
