

#ifndef __LINUX_OVCAMCHIP_PRIV_H
#define __LINUX_OVCAMCHIP_PRIV_H

#include <linux/i2c.h>
#include <media/v4l2-subdev.h>
#include <media/ovcamchip.h>

#ifdef DEBUG
extern int ovcamchip_debug;
#endif

#define PDEBUG(level, fmt, args...) \
	if (ovcamchip_debug >= (level))	pr_debug("[%s:%d] " fmt "\n", \
		__func__, __LINE__ , ## args)

#define DDEBUG(level, dev, fmt, args...) \
	if (ovcamchip_debug >= (level))	dev_dbg(dev, "[%s:%d] " fmt "\n", \
		__func__, __LINE__ , ## args)

#define I2C_DETECT_RETRIES	10

struct ovcamchip_regvals {
	unsigned char reg;
	unsigned char val;
};

struct ovcamchip_ops {
	int (*init)(struct i2c_client *);
	int (*free)(struct i2c_client *);
	int (*command)(struct i2c_client *, unsigned int, void *);
};

struct ovcamchip {
	struct v4l2_subdev sd;
	struct ovcamchip_ops *sops;
	void *spriv;               /* Private data for OV7x10.c etc... */
	int subtype;               /* = SEN_OV7610 etc... */
	int mono;                  /* Monochrome chip? (invalid until init) */
	int initialized;           /* OVCAMCHIP_CMD_INITIALIZE was successful */
};

static inline struct ovcamchip *to_ovcamchip(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ovcamchip, sd);
}

extern struct ovcamchip_ops ov6x20_ops;
extern struct ovcamchip_ops ov6x30_ops;
extern struct ovcamchip_ops ov7x10_ops;
extern struct ovcamchip_ops ov7x20_ops;
extern struct ovcamchip_ops ov76be_ops;

/* --------------------------------- */
/*              I2C I/O              */
/* --------------------------------- */

static inline int ov_read(struct i2c_client *c, unsigned char reg,
			  unsigned char *value)
{
	int rc;

	rc = i2c_smbus_read_byte_data(c, reg);
	*value = (unsigned char) rc;
	return rc;
}

static inline int ov_write(struct i2c_client *c, unsigned char reg,
			   unsigned char value )
{
	return i2c_smbus_write_byte_data(c, reg, value);
}

/* --------------------------------- */
/*        FUNCTION PROTOTYPES        */
/* --------------------------------- */

/* Functions in ovcamchip_core.c */

extern int ov_write_regvals(struct i2c_client *c,
			    struct ovcamchip_regvals *rvals);

extern int ov_write_mask(struct i2c_client *c, unsigned char reg,
			 unsigned char value, unsigned char mask);

#endif
