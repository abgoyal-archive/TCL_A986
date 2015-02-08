

#ifndef __AOA_H
#define __AOA_H
#include <asm/prom.h>
#include <linux/module.h>
#include <sound/core.h>
#include <sound/asound.h>
#include <sound/control.h>
#include "aoa-gpio.h"
#include "soundbus/soundbus.h"

#define MAX_CODEC_NAME_LEN	32

struct aoa_codec {
	char	name[MAX_CODEC_NAME_LEN];

	struct module *owner;

	/* called when the fabric wants to init this codec.
	 * Do alsa card manipulations from here. */
	int (*init)(struct aoa_codec *codec);

	/* called when the fabric is done with the codec.
	 * The alsa card will be cleaned up so don't bother. */
	void (*exit)(struct aoa_codec *codec);

	/* May be NULL, but can be used by the fabric.
	 * Refcounting is the codec driver's responsibility */
	struct device_node *node;

	/* assigned by fabric before init() is called, points
	 * to the soundbus device. Cannot be NULL. */
	struct soundbus_dev *soundbus_dev;

	/* assigned by the fabric before init() is called, points
	 * to the fabric's gpio runtime record for the relevant
	 * device. */
	struct gpio_runtime *gpio;

	/* assigned by the fabric before init() is called, contains
	 * a codec specific bitmask of what outputs and inputs are
	 * actually connected */
	u32 connected;

	/* data the fabric can associate with this structure */
	void *fabric_data;

	/* private! */
	struct list_head list;
	struct aoa_fabric *fabric;
};

/* return 0 on success */
extern int
aoa_codec_register(struct aoa_codec *codec);
extern void
aoa_codec_unregister(struct aoa_codec *codec);

#define MAX_LAYOUT_NAME_LEN	32

struct aoa_fabric {
	char	name[MAX_LAYOUT_NAME_LEN];

	struct module *owner;

	/* once codecs register, they are passed here after.
	 * They are of course not initialised, since the
	 * fabric is responsible for initialising some fields
	 * in the codec structure! */
	int (*found_codec)(struct aoa_codec *codec);
	/* called for each codec when it is removed,
	 * also in the case that aoa_fabric_unregister
	 * is called and all codecs are removed
	 * from this fabric.
	 * Also called if found_codec returned 0 but
	 * the codec couldn't initialise. */
	void (*remove_codec)(struct aoa_codec *codec);
	/* If found_codec returned 0, and the codec
	 * could be initialised, this is called. */
	void (*attached_codec)(struct aoa_codec *codec);
};

extern int
aoa_fabric_register(struct aoa_fabric *fabric, struct device *dev);

extern void
aoa_fabric_unregister(struct aoa_fabric *fabric);

extern void
aoa_fabric_unlink_codec(struct aoa_codec *codec);

/* alsa help methods */
struct aoa_card {
	struct snd_card *alsa_card;
};
        
extern int aoa_snd_device_new(snd_device_type_t type,
	void * device_data, struct snd_device_ops * ops);
extern struct snd_card *aoa_get_card(void);
extern int aoa_snd_ctl_add(struct snd_kcontrol* control);

/* GPIO stuff */
extern struct gpio_methods *pmf_gpio_methods;
extern struct gpio_methods *ftr_gpio_methods;
/* extern struct gpio_methods *map_gpio_methods; */

#endif /* __AOA_H */
