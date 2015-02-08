/* ------------------------------------------------------------------------- */
/* 									     */
/* i2c.h - definitions for the i2c-bus interface			     */
/* 									     */
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

/* --- For i2c-isa ---------------------------------------------------- */

extern void i2c_adapter_dev_release(struct device *dev);
extern struct device_driver i2c_adapter_driver;
extern struct class i2c_adapter_class;
extern struct bus_type i2c_bus_type;

/* --- General options ------------------------------------------------	*/

struct i2c_msg;
struct i2c_algorithm;
struct i2c_adapter;
struct i2c_client;
struct i2c_driver;
union i2c_smbus_data;

extern int i2c_master_send(struct i2c_client *,const char* ,int);
extern int i2c_master_recv(struct i2c_client *,char* ,int);

extern int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num);

extern int i2c_slave_send(struct i2c_client *,char*,int);
extern int i2c_slave_recv(struct i2c_client *,char*,int);



extern s32 i2c_smbus_xfer (struct i2c_adapter * adapter, u16 addr, 
                           unsigned short flags,
                           char read_write, u8 command, int size,
                           union i2c_smbus_data * data);


extern s32 i2c_smbus_write_quick(struct i2c_client * client, u8 value);
extern s32 i2c_smbus_read_byte(struct i2c_client * client);
extern s32 i2c_smbus_write_byte(struct i2c_client * client, u8 value);
extern s32 i2c_smbus_read_byte_data(struct i2c_client * client, u8 command);
extern s32 i2c_smbus_write_byte_data(struct i2c_client * client,
                                     u8 command, u8 value);
extern s32 i2c_smbus_read_word_data(struct i2c_client * client, u8 command);
extern s32 i2c_smbus_write_word_data(struct i2c_client * client,
                                     u8 command, u16 value);
extern s32 i2c_smbus_write_block_data(struct i2c_client * client,
				      u8 command, u8 length,
				      const u8 *values);
/* Returns the number of read bytes */
extern s32 i2c_smbus_read_i2c_block_data(struct i2c_client * client,
					 u8 command, u8 *values);
extern s32 i2c_smbus_write_i2c_block_data(struct i2c_client * client,
					  u8 command, u8 length,
					  const u8 *values);


struct i2c_driver {
	int id;
	unsigned int class;

	/* Notifies the driver that a new bus has appeared. This routine
	 * can be used by the driver to test if the bus meets its conditions
	 * & seek for the presence of the chip(s) it supports. If found, it 
	 * registers the client(s) that are on the bus to the i2c admin. via
	 * i2c_attach_client.
	 */
	int (*attach_adapter)(struct i2c_adapter *);
	int (*detach_adapter)(struct i2c_adapter *);

	/* tells the driver that a client is about to be deleted & gives it 
	 * the chance to remove its private data. Also, if the client struct
	 * has been dynamically allocated by the driver in the function above,
	 * it must be freed here.
	 */
	int (*detach_client)(struct i2c_client *);
	
	/* a ioctl like command that can be used to perform specific functions
	 * with the device.
	 */
	int (*command)(struct i2c_client *client,unsigned int cmd, void *arg);

	struct device_driver driver;
	struct list_head list;
};
#define to_i2c_driver(d) container_of(d, struct i2c_driver, driver)

#define I2C_NAME_SIZE	50

struct i2c_client {
	unsigned int flags;		/* div., see below		*/
	unsigned short addr;		/* chip address - NOTE: 7bit 	*/
					/* addresses are stored in the	*/
					/* _LOWER_ 7 bits		*/
	struct i2c_adapter *adapter;	/* the adapter we sit on	*/
	struct i2c_driver *driver;	/* and our access routines	*/
	int usage_count;		/* How many accesses currently  */
					/* to the client		*/
	struct device dev;		/* the device structure		*/
	struct list_head list;
	char name[I2C_NAME_SIZE];
	struct completion released;
};
#define to_i2c_client(d) container_of(d, struct i2c_client, dev)

static inline struct i2c_client *kobj_to_i2c_client(struct kobject *kobj)
{
	return to_i2c_client(container_of(kobj, struct device, kobj));
}

static inline void *i2c_get_clientdata (struct i2c_client *dev)
{
	return dev_get_drvdata (&dev->dev);
}

static inline void i2c_set_clientdata (struct i2c_client *dev, void *data)
{
	dev_set_drvdata (&dev->dev, data);
}

struct i2c_algorithm {
	/* If an adapter algorithm can't do I2C-level access, set master_xfer
	   to NULL. If an adapter algorithm can do SMBus access, set 
	   smbus_xfer. If set to NULL, the SMBus protocol is simulated
	   using common I2C messages */
	/* master_xfer should return the number of messages successfully
	   processed, or a negative value on error */
	int (*master_xfer)(struct i2c_adapter *adap,struct i2c_msg *msgs, 
	                   int num);
	int (*smbus_xfer) (struct i2c_adapter *adap, u16 addr, 
	                   unsigned short flags, char read_write,
	                   u8 command, int size, union i2c_smbus_data * data);

	/* --- these optional/future use for some adapter types.*/
	int (*slave_send)(struct i2c_adapter *,char*,int);
	int (*slave_recv)(struct i2c_adapter *,char*,int);

	/* --- ioctl like call to set div. parameters. */
	int (*algo_control)(struct i2c_adapter *, unsigned int, unsigned long);

	/* To determine what the adapter supports */
	u32 (*functionality) (struct i2c_adapter *);
};

struct i2c_adapter {
	struct module *owner;
	unsigned int id;
	unsigned int class;
	struct i2c_algorithm *algo;/* the algorithm to access the bus	*/
	void *algo_data;

	/* --- administration stuff. */
	int (*client_register)(struct i2c_client *);
	int (*client_unregister)(struct i2c_client *);

	/* data fields that are valid for all devices	*/
	struct mutex bus_lock;
	struct mutex clist_lock;

	int timeout;
	int retries;
	struct device dev;		/* the adapter device */
	struct class_device class_dev;	/* the class device */

	int nr;
	struct list_head clients;
	struct list_head list;
	char name[I2C_NAME_SIZE];
	struct completion dev_released;
	struct completion class_dev_released;
};
#define dev_to_i2c_adapter(d) container_of(d, struct i2c_adapter, dev)
#define class_dev_to_i2c_adapter(d) container_of(d, struct i2c_adapter, class_dev)

static inline void *i2c_get_adapdata (struct i2c_adapter *dev)
{
	return dev_get_drvdata (&dev->dev);
}

static inline void i2c_set_adapdata (struct i2c_adapter *dev, void *data)
{
	dev_set_drvdata (&dev->dev, data);
}

/*flags for the client struct: */
#define I2C_CLIENT_PEC  0x04			/* Use Packet Error Checking */
#define I2C_CLIENT_TEN	0x10			/* we have a ten bit chip address	*/
						/* Must equal I2C_M_TEN below */

/* i2c adapter classes (bitmask) */
#define I2C_CLASS_HWMON		(1<<0)	/* lm_sensors, ... */
#define I2C_CLASS_TV_ANALOG	(1<<1)	/* bttv + friends */
#define I2C_CLASS_TV_DIGITAL	(1<<2)	/* dvb cards */
#define I2C_CLASS_DDC		(1<<3)	/* i2c-matroxfb ? */
#define I2C_CLASS_CAM_ANALOG	(1<<4)	/* camera with analog CCD */
#define I2C_CLASS_CAM_DIGITAL	(1<<5)	/* most webcams */
#define I2C_CLASS_SOUND		(1<<6)	/* sound devices */
#define I2C_CLASS_ALL		(UINT_MAX) /* all of the above */

struct i2c_client_address_data {
	unsigned short *normal_i2c;
	unsigned short *probe;
	unsigned short *ignore;
	unsigned short **forces;
};

/* Internal numbers to terminate lists */
#define I2C_CLIENT_END		0xfffeU

/* The numbers to use to set I2C bus address */
#define ANY_I2C_BUS		0xffff
#define ANY_I2C_ISA_BUS		9191


/* ----- functions exported by i2c.o */

extern int i2c_add_adapter(struct i2c_adapter *);
extern int i2c_del_adapter(struct i2c_adapter *);

extern int i2c_register_driver(struct module *, struct i2c_driver *);
extern int i2c_del_driver(struct i2c_driver *);

static inline int i2c_add_driver(struct i2c_driver *driver)
{
	return i2c_register_driver(THIS_MODULE, driver);
}

extern int i2c_attach_client(struct i2c_client *);
extern int i2c_detach_client(struct i2c_client *);

extern int i2c_use_client(struct i2c_client *);
extern int i2c_release_client(struct i2c_client *);

extern void i2c_clients_command(struct i2c_adapter *adap,
				unsigned int cmd, void *arg);

extern int i2c_check_addr (struct i2c_adapter *adapter, int addr);

extern int i2c_probe(struct i2c_adapter *adapter, 
		struct i2c_client_address_data *address_data,
		int (*found_proc) (struct i2c_adapter *, int, int));

extern int i2c_control(struct i2c_client *,unsigned int, unsigned long);

extern struct i2c_adapter* i2c_get_adapter(int id);
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

/* Return id number for a specific adapter */
static inline int i2c_adapter_id(struct i2c_adapter *adap)
{
	return adap->nr;
}
#endif /* __KERNEL__ */

struct i2c_msg {
	__u16 addr;	/* slave address			*/
 	__u16 flags;		
#define I2C_M_TEN	0x10	/* we have a ten bit chip address	*/
#define I2C_M_RD	0x01
#define I2C_M_NOSTART	0x4000
#define I2C_M_REV_DIR_ADDR	0x2000
#define I2C_M_IGNORE_NAK	0x1000
#define I2C_M_NO_RD_ACK		0x0800
 	__u16 len;		/* msg length				*/
 	__u8 *buf;		/* pointer to msg data			*/
};

/* To determine what functionality is present */

#define I2C_FUNC_I2C			0x00000001
#define I2C_FUNC_10BIT_ADDR		0x00000002
#define I2C_FUNC_PROTOCOL_MANGLING	0x00000004 /* I2C_M_{REV_DIR_ADDR,NOSTART,..} */
#define I2C_FUNC_SMBUS_HWPEC_CALC	0x00000008 /* SMBus 2.0 */
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
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK_2	 0x10000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2 0x20000000 /* w/ 2-byte reg. addr. */

#define I2C_FUNC_SMBUS_BYTE (I2C_FUNC_SMBUS_READ_BYTE | \
                             I2C_FUNC_SMBUS_WRITE_BYTE)
#define I2C_FUNC_SMBUS_BYTE_DATA (I2C_FUNC_SMBUS_READ_BYTE_DATA | \
                                  I2C_FUNC_SMBUS_WRITE_BYTE_DATA)
#define I2C_FUNC_SMBUS_WORD_DATA (I2C_FUNC_SMBUS_READ_WORD_DATA | \
                                  I2C_FUNC_SMBUS_WRITE_WORD_DATA)
#define I2C_FUNC_SMBUS_BLOCK_DATA (I2C_FUNC_SMBUS_READ_BLOCK_DATA | \
                                   I2C_FUNC_SMBUS_WRITE_BLOCK_DATA)
#define I2C_FUNC_SMBUS_I2C_BLOCK (I2C_FUNC_SMBUS_READ_I2C_BLOCK | \
                                  I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)
#define I2C_FUNC_SMBUS_I2C_BLOCK_2 (I2C_FUNC_SMBUS_READ_I2C_BLOCK_2 | \
                                    I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2)

#define I2C_FUNC_SMBUS_EMUL (I2C_FUNC_SMBUS_QUICK | \
                             I2C_FUNC_SMBUS_BYTE | \
                             I2C_FUNC_SMBUS_BYTE_DATA | \
                             I2C_FUNC_SMBUS_WORD_DATA | \
                             I2C_FUNC_SMBUS_PROC_CALL | \
                             I2C_FUNC_SMBUS_WRITE_BLOCK_DATA | \
                             I2C_FUNC_SMBUS_I2C_BLOCK)

#define I2C_SMBUS_BLOCK_MAX	32	/* As specified in SMBus standard */	
union i2c_smbus_data {
	__u8 byte;
	__u16 word;
	__u8 block[I2C_SMBUS_BLOCK_MAX + 2]; /* block[0] is used for length */
	                       /* and one more for user-space compatibility */
};

/* smbus_access read or write markers */
#define I2C_SMBUS_READ	1
#define I2C_SMBUS_WRITE	0

#define I2C_SMBUS_QUICK		    0
#define I2C_SMBUS_BYTE		    1
#define I2C_SMBUS_BYTE_DATA	    2 
#define I2C_SMBUS_WORD_DATA	    3
#define I2C_SMBUS_PROC_CALL	    4
#define I2C_SMBUS_BLOCK_DATA	    5
#define I2C_SMBUS_I2C_BLOCK_DATA    6
#define I2C_SMBUS_BLOCK_PROC_CALL   7		/* SMBus 2.0 */


				/* -> bit-adapter specific ioctls	*/
#define I2C_RETRIES	0x0701	/* number of times a device address      */
				/* should be polled when not            */
                                /* acknowledging 			*/
#define I2C_TIMEOUT	0x0702	/* set timeout - call with int 		*/


/* this is for i2c-dev.c	*/
#define I2C_SLAVE	0x0703	/* Change slave address			*/
				/* Attn.: Slave address is 7 or 10 bits */
#define I2C_SLAVE_FORCE	0x0706	/* Change slave address			*/
				/* Attn.: Slave address is 7 or 10 bits */
				/* This changes the address, even if it */
				/* is already taken!			*/
#define I2C_TENBIT	0x0704	/* 0 for 7 bit addrs, != 0 for 10 bit	*/

#define I2C_FUNCS	0x0705	/* Get the adapter functionality */
#define I2C_RDWR	0x0707	/* Combined R/W transfer (one stop only)*/
#define I2C_PEC		0x0708	/* != 0 for SMBus PEC                   */

#define I2C_SMBUS	0x0720	/* SMBus-level access */

/* ----- I2C-DEV: char device interface stuff ------------------------- */
#ifdef __KERNEL__

#define I2C_MAJOR	89		/* Device major number		*/

/* These defines are used for probing i2c client addresses */
/* The length of the option lists */
#define I2C_CLIENT_MAX_OPTS 48

/* Default fill of many variables */
#define I2C_CLIENT_DEFAULTS {I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, \
                          I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END}


#define I2C_CLIENT_MODULE_PARM(var,desc) \
  static unsigned short var[I2C_CLIENT_MAX_OPTS] = I2C_CLIENT_DEFAULTS; \
  static unsigned int var##_num; \
  module_param_array(var, short, &var##_num, 0); \
  MODULE_PARM_DESC(var,desc)

#define I2C_CLIENT_MODULE_PARM_FORCE(name)				\
I2C_CLIENT_MODULE_PARM(force_##name,					\
		       "List of adapter,address pairs which are "	\
		       "unquestionably assumed to contain a `"		\
		       # name "' chip")


#define I2C_CLIENT_INSMOD_COMMON					\
I2C_CLIENT_MODULE_PARM(probe, "List of adapter,address pairs to scan "	\
		       "additionally");					\
I2C_CLIENT_MODULE_PARM(ignore, "List of adapter,address pairs not to "	\
		       "scan");						\
static struct i2c_client_address_data addr_data = {			\
	.normal_i2c	= normal_i2c,					\
	.probe		= probe,					\
	.ignore		= ignore,					\
	.forces		= forces,					\
}

#define I2C_CLIENT_INSMOD \
  I2C_CLIENT_MODULE_PARM(force, \
                      "List of adapter,address pairs to boldly assume " \
                      "to be present"); \
	static unsigned short *forces[] = {				\
			force,						\
			NULL						\
		};							\
I2C_CLIENT_INSMOD_COMMON

#define I2C_CLIENT_INSMOD_1(chip1)					\
enum chips { any_chip, chip1 };						\
I2C_CLIENT_MODULE_PARM(force, "List of adapter,address pairs to "	\
		       "boldly assume to be present");			\
I2C_CLIENT_MODULE_PARM_FORCE(chip1);					\
static unsigned short *forces[] = { force, force_##chip1, NULL };	\
I2C_CLIENT_INSMOD_COMMON

#define I2C_CLIENT_INSMOD_2(chip1, chip2)				\
enum chips { any_chip, chip1, chip2 };					\
I2C_CLIENT_MODULE_PARM(force, "List of adapter,address pairs to "	\
		       "boldly assume to be present");			\
I2C_CLIENT_MODULE_PARM_FORCE(chip1);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip2);					\
static unsigned short *forces[] = { force, force_##chip1,		\
				    force_##chip2, NULL };		\
I2C_CLIENT_INSMOD_COMMON

#define I2C_CLIENT_INSMOD_3(chip1, chip2, chip3)			\
enum chips { any_chip, chip1, chip2, chip3 };				\
I2C_CLIENT_MODULE_PARM(force, "List of adapter,address pairs to "	\
		       "boldly assume to be present");			\
I2C_CLIENT_MODULE_PARM_FORCE(chip1);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip2);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip3);					\
static unsigned short *forces[] = { force, force_##chip1,		\
				    force_##chip2, force_##chip3,	\
				    NULL };				\
I2C_CLIENT_INSMOD_COMMON

#define I2C_CLIENT_INSMOD_4(chip1, chip2, chip3, chip4)			\
enum chips { any_chip, chip1, chip2, chip3, chip4 };			\
I2C_CLIENT_MODULE_PARM(force, "List of adapter,address pairs to "	\
		       "boldly assume to be present");			\
I2C_CLIENT_MODULE_PARM_FORCE(chip1);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip2);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip3);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip4);					\
static unsigned short *forces[] = { force, force_##chip1,		\
				    force_##chip2, force_##chip3,	\
				    force_##chip4, NULL};		\
I2C_CLIENT_INSMOD_COMMON

#define I2C_CLIENT_INSMOD_5(chip1, chip2, chip3, chip4, chip5)		\
enum chips { any_chip, chip1, chip2, chip3, chip4, chip5 };		\
I2C_CLIENT_MODULE_PARM(force, "List of adapter,address pairs to "	\
		       "boldly assume to be present");			\
I2C_CLIENT_MODULE_PARM_FORCE(chip1);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip2);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip3);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip4);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip5);					\
static unsigned short *forces[] = { force, force_##chip1,		\
				    force_##chip2, force_##chip3,	\
				    force_##chip4, force_##chip5,	\
				    NULL };				\
I2C_CLIENT_INSMOD_COMMON

#define I2C_CLIENT_INSMOD_6(chip1, chip2, chip3, chip4, chip5, chip6)	\
enum chips { any_chip, chip1, chip2, chip3, chip4, chip5, chip6 };	\
I2C_CLIENT_MODULE_PARM(force, "List of adapter,address pairs to "	\
		       "boldly assume to be present");			\
I2C_CLIENT_MODULE_PARM_FORCE(chip1);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip2);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip3);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip4);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip5);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip6);					\
static unsigned short *forces[] = { force, force_##chip1,		\
				    force_##chip2, force_##chip3,	\
				    force_##chip4, force_##chip5,	\
				    force_##chip6, NULL };		\
I2C_CLIENT_INSMOD_COMMON

#define I2C_CLIENT_INSMOD_7(chip1, chip2, chip3, chip4, chip5, chip6, chip7) \
enum chips { any_chip, chip1, chip2, chip3, chip4, chip5, chip6,	\
	     chip7 };							\
I2C_CLIENT_MODULE_PARM(force, "List of adapter,address pairs to "	\
		       "boldly assume to be present");			\
I2C_CLIENT_MODULE_PARM_FORCE(chip1);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip2);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip3);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip4);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip5);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip6);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip7);					\
static unsigned short *forces[] = { force, force_##chip1,		\
				    force_##chip2, force_##chip3,	\
				    force_##chip4, force_##chip5,	\
				    force_##chip6, force_##chip7,	\
				    NULL };				\
I2C_CLIENT_INSMOD_COMMON

#define I2C_CLIENT_INSMOD_8(chip1, chip2, chip3, chip4, chip5, chip6, chip7, chip8) \
enum chips { any_chip, chip1, chip2, chip3, chip4, chip5, chip6,	\
	     chip7, chip8 };						\
I2C_CLIENT_MODULE_PARM(force, "List of adapter,address pairs to "	\
		       "boldly assume to be present");			\
I2C_CLIENT_MODULE_PARM_FORCE(chip1);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip2);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip3);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip4);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip5);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip6);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip7);					\
I2C_CLIENT_MODULE_PARM_FORCE(chip8);					\
static unsigned short *forces[] = { force, force_##chip1,		\
				    force_##chip2, force_##chip3,	\
				    force_##chip4, force_##chip5,	\
				    force_##chip6, force_##chip7,	\
				    force_##chip8, NULL };		\
I2C_CLIENT_INSMOD_COMMON
#endif /* __KERNEL__ */
#endif /* _LINUX_I2C_H */
