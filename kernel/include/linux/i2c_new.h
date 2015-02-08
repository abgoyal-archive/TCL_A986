
/* ------------------------------------------------------------------------- */
/*									     */
/* i2c.h - definitions for the i2c-bus interface			     */
/*									     */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


#ifndef _LINUX_I2C_H
#define _LINUX_I2C_H

#include <linux/types.h>
#ifdef __KERNEL__
#include <linux/module.h>
#include <linux/i2c-id.h>
#include <linux/mod_devicetable.h>
#include <linux/device.h>	/* for struct device */
#include <linux/sched.h>	/* for completion */
#include <linux/mutex.h>
#include <linux/of.h>		/* for struct device_node */

extern struct bus_type i2c_bus_type;

/* --- General options ------------------------------------------------	*/

struct i2c_msg;
struct i2c_algorithm;
struct i2c_adapter;
struct i2c_client;
struct i2c_driver;
union i2c_smbus_data;
struct i2c_board_info;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
extern int i2c_master_send(struct i2c_client *client, const char *buf,
			   int count);
extern int i2c_master_recv(struct i2c_client *client, char *buf, int count);

extern int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			int num);

extern s32 i2c_smbus_xfer(struct i2c_adapter *adapter, u16 addr,
			  unsigned short flags, char read_write, u8 command,
			  int size, union i2c_smbus_data *data);


extern s32 i2c_smbus_read_byte(struct i2c_client *client);
extern s32 i2c_smbus_write_byte(struct i2c_client *client, u8 value);
extern s32 i2c_smbus_read_byte_data(struct i2c_client *client, u8 command);
extern s32 i2c_smbus_write_byte_data(struct i2c_client *client,
				     u8 command, u8 value);
extern s32 i2c_smbus_read_word_data(struct i2c_client *client, u8 command);
extern s32 i2c_smbus_write_word_data(struct i2c_client *client,
				     u8 command, u16 value);
/* Returns the number of read bytes */
extern s32 i2c_smbus_read_block_data(struct i2c_client *client,
				     u8 command, u8 *values);
extern s32 i2c_smbus_write_block_data(struct i2c_client *client,
				      u8 command, u8 length, const u8 *values);
/* Returns the number of read bytes */
extern s32 i2c_smbus_read_i2c_block_data(struct i2c_client *client,
					 u8 command, u8 length, u8 *values);
extern s32 i2c_smbus_write_i2c_block_data(struct i2c_client *client,
					  u8 command, u8 length,
					  const u8 *values);
#endif /* I2C */

struct i2c_driver {
	unsigned int class;

	/* Notifies the driver that a new bus has appeared or is about to be
	 * removed. You should avoid using this if you can, it will probably
	 * be removed in a near future.
	 */
	int (*attach_adapter)(struct i2c_adapter *);
	int (*detach_adapter)(struct i2c_adapter *);

	/* Standard driver model interfaces */
	int (*probe)(struct i2c_client *, const struct i2c_device_id *);
	int (*remove)(struct i2c_client *);

	/* driver model interfaces that don't relate to enumeration  */
	void (*shutdown)(struct i2c_client *);
	int (*suspend)(struct i2c_client *, pm_message_t mesg);
	int (*resume)(struct i2c_client *);

	/* Alert callback, for example for the SMBus alert protocol.
	 * The format and meaning of the data value depends on the protocol.
	 * For the SMBus alert protocol, there is a single bit of data passed
	 * as the alert response's low bit ("event flag").
	 */
	void (*alert)(struct i2c_client *, unsigned int data);

	/* a ioctl like command that can be used to perform specific functions
	 * with the device.
	 */
	int (*command)(struct i2c_client *client, unsigned int cmd, void *arg);

	struct device_driver driver;
	const struct i2c_device_id *id_table;

	/* Device detection callback for automatic device creation */
	int (*detect)(struct i2c_client *, struct i2c_board_info *);
	const unsigned short *address_list;
	struct list_head clients;
};
#define to_i2c_driver(d) container_of(d, struct i2c_driver, driver)

struct i2c_client {
	unsigned short flags;		/* div., see below		*/
	unsigned short addr;		/* chip address - NOTE: 7bit	*/
					/* addresses are stored in the	*/
					/* _LOWER_ 7 bits		*/
	char name[I2C_NAME_SIZE];
	struct i2c_adapter *adapter;	/* the adapter we sit on	*/
	struct i2c_driver *driver;	/* and our access routines	*/
	struct device dev;		/* the device structure		*/
	int irq;			/* irq issued by device		*/
	struct list_head detected;
};
#define to_i2c_client(d) container_of(d, struct i2c_client, dev)

extern struct i2c_client *i2c_verify_client(struct device *dev);

static inline struct i2c_client *kobj_to_i2c_client(struct kobject *kobj)
{
	struct device * const dev = container_of(kobj, struct device, kobj);
	return to_i2c_client(dev);
}

static inline void *i2c_get_clientdata(const struct i2c_client *dev)
{
	return dev_get_drvdata(&dev->dev);
}

static inline void i2c_set_clientdata(struct i2c_client *dev, void *data)
{
	dev_set_drvdata(&dev->dev, data);
}

struct i2c_board_info {
	char		type[I2C_NAME_SIZE];
	unsigned short	flags;
	unsigned short	addr;
	void		*platform_data;
	struct dev_archdata	*archdata;
#ifdef CONFIG_OF
	struct device_node *of_node;
#endif
	int		irq;
};

#define I2C_BOARD_INFO(dev_type, dev_addr) \
	.type = dev_type, .addr = (dev_addr)


#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
extern struct i2c_client *
i2c_new_device(struct i2c_adapter *adap, struct i2c_board_info const *info);

extern struct i2c_client *
i2c_new_probed_device(struct i2c_adapter *adap,
		      struct i2c_board_info *info,
		      unsigned short const *addr_list);

extern struct i2c_client *
i2c_new_dummy(struct i2c_adapter *adap, u16 address);

extern void i2c_unregister_device(struct i2c_client *);
#endif /* I2C */

#ifdef CONFIG_I2C_BOARDINFO
extern int
i2c_register_board_info(int busnum, struct i2c_board_info const *info,
			unsigned n);
#else
static inline int
i2c_register_board_info(int busnum, struct i2c_board_info const *info,
			unsigned n)
{
	return 0;
}
#endif /* I2C_BOARDINFO */

struct i2c_algorithm {
	/* If an adapter algorithm can't do I2C-level access, set master_xfer
	   to NULL. If an adapter algorithm can do SMBus access, set
	   smbus_xfer. If set to NULL, the SMBus protocol is simulated
	   using common I2C messages */
	/* master_xfer should return the number of messages successfully
	   processed, or a negative value on error */
	int (*master_xfer)(struct i2c_adapter *adap, struct i2c_msg *msgs,
			   int num);
	int (*smbus_xfer) (struct i2c_adapter *adap, u16 addr,
			   unsigned short flags, char read_write,
			   u8 command, int size, union i2c_smbus_data *data);

	/* To determine what the adapter supports */
	u32 (*functionality) (struct i2c_adapter *);
};

struct i2c_adapter {
	struct module *owner;
	unsigned int id;
	unsigned int class;		  /* classes to allow probing for */
	const struct i2c_algorithm *algo; /* the algorithm to access the bus */
	void *algo_data;

	/* data fields that are valid for all devices	*/
	struct rt_mutex bus_lock;

	int timeout;			/* in jiffies */
	int retries;
	struct device dev;		/* the adapter device */

	int nr;
	char name[48];
	struct completion dev_released;

	struct list_head userspace_clients;
};
#define to_i2c_adapter(d) container_of(d, struct i2c_adapter, dev)

static inline void *i2c_get_adapdata(const struct i2c_adapter *dev)
{
	return dev_get_drvdata(&dev->dev);
}

static inline void i2c_set_adapdata(struct i2c_adapter *dev, void *data)
{
	dev_set_drvdata(&dev->dev, data);
}

static inline void i2c_lock_adapter(struct i2c_adapter *adapter)
{
	rt_mutex_lock(&adapter->bus_lock);
}

static inline void i2c_unlock_adapter(struct i2c_adapter *adapter)
{
	rt_mutex_unlock(&adapter->bus_lock);
}

/*flags for the client struct: */
#define I2C_CLIENT_PEC	0x04		/* Use Packet Error Checking */
#define I2C_CLIENT_TEN	0x10		/* we have a ten bit chip address */
					/* Must equal I2C_M_TEN below */
#define I2C_CLIENT_WAKE	0x80		/* for board_info; true iff can wake */

/* i2c adapter classes (bitmask) */
#define I2C_CLASS_HWMON		(1<<0)	/* lm_sensors, ... */
#define I2C_CLASS_TV_ANALOG	(1<<1)	/* bttv + friends */
#define I2C_CLASS_TV_DIGITAL	(1<<2)	/* dvb cards */
#define I2C_CLASS_DDC		(1<<3)	/* DDC bus on graphics adapters */
#define I2C_CLASS_SPD		(1<<7)	/* SPD EEPROMs and similar */

/* Internal numbers to terminate lists */
#define I2C_CLIENT_END		0xfffeU

/* The numbers to use to set I2C bus address */
#define ANY_I2C_BUS		0xffff

/* Construct an I2C_CLIENT_END-terminated array of i2c addresses */
#define I2C_ADDRS(addr, addrs...) \
	((const unsigned short []){ addr, ## addrs, I2C_CLIENT_END })


/* ----- functions exported by i2c.o */

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
extern int i2c_add_adapter(struct i2c_adapter *);
extern int i2c_del_adapter(struct i2c_adapter *);
extern int i2c_add_numbered_adapter(struct i2c_adapter *);

extern int i2c_register_driver(struct module *, struct i2c_driver *);
extern void i2c_del_driver(struct i2c_driver *);

static inline int i2c_add_driver(struct i2c_driver *driver)
{
	return i2c_register_driver(THIS_MODULE, driver);
}

extern struct i2c_client *i2c_use_client(struct i2c_client *client);
extern void i2c_release_client(struct i2c_client *client);

extern void i2c_clients_command(struct i2c_adapter *adap,
				unsigned int cmd, void *arg);

extern struct i2c_adapter *i2c_get_adapter(int id);
extern void i2c_put_adapter(struct i2c_adapter *adap);


/* Return the functionality mask */
static inline u32 i2c_get_functionality(struct i2c_adapter *adap)
{
	return adap->algo->functionality(adap);
}

/* Return 1 if adapter supports everything we need, 0 if not. */
static inline int i2c_check_functionality(struct i2c_adapter *adap, u32 func)
{
	return (func & i2c_get_functionality(adap)) == func;
}

/* Return the adapter number for a specific adapter */
static inline int i2c_adapter_id(struct i2c_adapter *adap)
{
	return adap->nr;
}
#endif /* I2C */
#endif /* __KERNEL__ */

struct i2c_msg {
	__u16 addr;	/* slave address			*/
	__u16 flags;
#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2C_M_RD		0x0001	/* read data, from slave to master */
#define I2C_M_NOSTART		0x4000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_REV_DIR_ADDR	0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK		0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */
	__u16 len;		/* msg length				*/
	__u8 *buf;		/* pointer to msg data			*/
};

/* To determine what functionality is present */

#define I2C_FUNC_I2C			0x00000001
#define I2C_FUNC_10BIT_ADDR		0x00000002
#define I2C_FUNC_PROTOCOL_MANGLING	0x00000004 /* I2C_M_NOSTART etc. */
#define I2C_FUNC_SMBUS_PEC		0x00000008
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL	0x00008000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_QUICK		0x00010000
#define I2C_FUNC_SMBUS_READ_BYTE	0x00020000
#define I2C_FUNC_SMBUS_WRITE_BYTE	0x00040000
#define I2C_FUNC_SMBUS_READ_BYTE_DATA	0x00080000
#define I2C_FUNC_SMBUS_WRITE_BYTE_DATA	0x00100000
#define I2C_FUNC_SMBUS_READ_WORD_DATA	0x00200000
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA	0x00400000
#define I2C_FUNC_SMBUS_PROC_CALL	0x00800000
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA	0x01000000
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA 0x02000000
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK	0x04000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK	0x08000000 /* w/ 1-byte reg. addr. */

#define I2C_FUNC_SMBUS_BYTE		(I2C_FUNC_SMBUS_READ_BYTE | \
					 I2C_FUNC_SMBUS_WRITE_BYTE)
#define I2C_FUNC_SMBUS_BYTE_DATA	(I2C_FUNC_SMBUS_READ_BYTE_DATA | \
					 I2C_FUNC_SMBUS_WRITE_BYTE_DATA)
#define I2C_FUNC_SMBUS_WORD_DATA	(I2C_FUNC_SMBUS_READ_WORD_DATA | \
					 I2C_FUNC_SMBUS_WRITE_WORD_DATA)
#define I2C_FUNC_SMBUS_BLOCK_DATA	(I2C_FUNC_SMBUS_READ_BLOCK_DATA | \
					 I2C_FUNC_SMBUS_WRITE_BLOCK_DATA)
#define I2C_FUNC_SMBUS_I2C_BLOCK	(I2C_FUNC_SMBUS_READ_I2C_BLOCK | \
					 I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)

#define I2C_FUNC_SMBUS_EMUL		(I2C_FUNC_SMBUS_QUICK | \
					 I2C_FUNC_SMBUS_BYTE | \
					 I2C_FUNC_SMBUS_BYTE_DATA | \
					 I2C_FUNC_SMBUS_WORD_DATA | \
					 I2C_FUNC_SMBUS_PROC_CALL | \
					 I2C_FUNC_SMBUS_WRITE_BLOCK_DATA | \
					 I2C_FUNC_SMBUS_I2C_BLOCK | \
					 I2C_FUNC_SMBUS_PEC)

#define I2C_SMBUS_BLOCK_MAX	32	/* As specified in SMBus standard */
union i2c_smbus_data {
	__u8 byte;
	__u16 word;
	__u8 block[I2C_SMBUS_BLOCK_MAX + 2]; /* block[0] is used for length */
			       /* and one more for user-space compatibility */
};

/* i2c_smbus_xfer read or write markers */
#define I2C_SMBUS_READ	1
#define I2C_SMBUS_WRITE	0

#define I2C_SMBUS_QUICK		    0
#define I2C_SMBUS_BYTE		    1
#define I2C_SMBUS_BYTE_DATA	    2
#define I2C_SMBUS_WORD_DATA	    3
#define I2C_SMBUS_PROC_CALL	    4
#define I2C_SMBUS_BLOCK_DATA	    5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7		/* SMBus 2.0 */
#define I2C_SMBUS_I2C_BLOCK_DATA    8

#endif /* _LINUX_I2C_H */
