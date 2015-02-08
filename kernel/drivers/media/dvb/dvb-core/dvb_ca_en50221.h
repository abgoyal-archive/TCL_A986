

#ifndef _DVB_CA_EN50221_H_
#define _DVB_CA_EN50221_H_

#include <linux/list.h>
#include <linux/dvb/ca.h>

#include "dvbdev.h"

#define DVB_CA_EN50221_POLL_CAM_PRESENT	1
#define DVB_CA_EN50221_POLL_CAM_CHANGED	2
#define DVB_CA_EN50221_POLL_CAM_READY		4

#define DVB_CA_EN50221_FLAG_IRQ_CAMCHANGE	1
#define DVB_CA_EN50221_FLAG_IRQ_FR		2
#define DVB_CA_EN50221_FLAG_IRQ_DA		4

#define DVB_CA_EN50221_CAMCHANGE_REMOVED		0
#define DVB_CA_EN50221_CAMCHANGE_INSERTED		1



/* Structure describing a CA interface */
struct dvb_ca_en50221 {

	/* the module owning this structure */
	struct module* owner;

	/* NOTE: the read_*, write_* and poll_slot_status functions will be
	 * called for different slots concurrently and need to use locks where
	 * and if appropriate. There will be no concurrent access to one slot.
	 */

	/* functions for accessing attribute memory on the CAM */
	int (*read_attribute_mem)(struct dvb_ca_en50221* ca, int slot, int address);
	int (*write_attribute_mem)(struct dvb_ca_en50221* ca, int slot, int address, u8 value);

	/* functions for accessing the control interface on the CAM */
	int (*read_cam_control)(struct dvb_ca_en50221* ca, int slot, u8 address);
	int (*write_cam_control)(struct dvb_ca_en50221* ca, int slot, u8 address, u8 value);

	/* Functions for controlling slots */
	int (*slot_reset)(struct dvb_ca_en50221* ca, int slot);
	int (*slot_shutdown)(struct dvb_ca_en50221* ca, int slot);
	int (*slot_ts_enable)(struct dvb_ca_en50221* ca, int slot);

	/*
	* Poll slot status.
	* Only necessary if DVB_CA_FLAG_EN50221_IRQ_CAMCHANGE is not set
	*/
	int (*poll_slot_status)(struct dvb_ca_en50221* ca, int slot, int open);

	/* private data, used by caller */
	void* data;

	/* Opaque data used by the dvb_ca core. Do not modify! */
	void* private;
};




/* ******************************************************************************** */
/* Functions for reporting IRQ events */

void dvb_ca_en50221_camchange_irq(struct dvb_ca_en50221* pubca, int slot, int change_type);

void dvb_ca_en50221_camready_irq(struct dvb_ca_en50221* pubca, int slot);

void dvb_ca_en50221_frda_irq(struct dvb_ca_en50221* ca, int slot);



/* ******************************************************************************** */
/* Initialisation/shutdown functions */

extern int dvb_ca_en50221_init(struct dvb_adapter *dvb_adapter, struct dvb_ca_en50221* ca, int flags, int slot_count);

extern void dvb_ca_en50221_release(struct dvb_ca_en50221* ca);



#endif
