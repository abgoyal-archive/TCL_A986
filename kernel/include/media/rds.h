

#ifndef _RDS_H
#define _RDS_H

struct rds_command {
	unsigned int  block_count;
	int           result;
	unsigned char __user *buffer;
	struct file   *instance;
	poll_table    *event_list;
};

#define RDS_CMD_OPEN	_IOW('R',1,int)
#define RDS_CMD_CLOSE	_IOW('R',2,int)
#define RDS_CMD_READ	_IOR('R',3,int)
#define RDS_CMD_POLL	_IOR('R',4,int)

#endif
