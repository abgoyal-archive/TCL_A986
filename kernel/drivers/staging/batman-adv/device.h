

#include "types.h"

void bat_device_init(void);
int bat_device_setup(void);
void bat_device_destroy(void);
int bat_device_open(struct inode *inode, struct file *file);
int bat_device_release(struct inode *inode, struct file *file);
ssize_t bat_device_read(struct file *file, char __user *buf, size_t count,
			loff_t *ppos);
ssize_t bat_device_write(struct file *file, const char __user *buff,
			 size_t len, loff_t *off);
unsigned int bat_device_poll(struct file *file, poll_table *wait);
void bat_device_add_packet(struct device_client *device_client,
			   struct icmp_packet *icmp_packet);
void bat_device_receive_packet(struct icmp_packet *icmp_packet);
