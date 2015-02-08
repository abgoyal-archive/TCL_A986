

#ifndef __XEN_PUBLIC_IO_KBDIF_H__
#define __XEN_PUBLIC_IO_KBDIF_H__

/* In events (backend -> frontend) */


/* Pointer movement event */
#define XENKBD_TYPE_MOTION  1
/* Event type 2 currently not used */
/* Key event (includes pointer buttons) */
#define XENKBD_TYPE_KEY     3
#define XENKBD_TYPE_POS     4

struct xenkbd_motion {
	uint8_t type;		/* XENKBD_TYPE_MOTION */
	int32_t rel_x;		/* relative X motion */
	int32_t rel_y;		/* relative Y motion */
	int32_t rel_z;		/* relative Z motion (wheel) */
};

struct xenkbd_key {
	uint8_t type;		/* XENKBD_TYPE_KEY */
	uint8_t pressed;	/* 1 if pressed; 0 otherwise */
	uint32_t keycode;	/* KEY_* from linux/input.h */
};

struct xenkbd_position {
	uint8_t type;		/* XENKBD_TYPE_POS */
	int32_t abs_x;		/* absolute X position (in FB pixels) */
	int32_t abs_y;		/* absolute Y position (in FB pixels) */
	int32_t rel_z;		/* relative Z motion (wheel) */
};

#define XENKBD_IN_EVENT_SIZE 40

union xenkbd_in_event {
	uint8_t type;
	struct xenkbd_motion motion;
	struct xenkbd_key key;
	struct xenkbd_position pos;
	char pad[XENKBD_IN_EVENT_SIZE];
};

/* Out events (frontend -> backend) */


#define XENKBD_OUT_EVENT_SIZE 40

union xenkbd_out_event {
	uint8_t type;
	char pad[XENKBD_OUT_EVENT_SIZE];
};

/* shared page */

#define XENKBD_IN_RING_SIZE 2048
#define XENKBD_IN_RING_LEN (XENKBD_IN_RING_SIZE / XENKBD_IN_EVENT_SIZE)
#define XENKBD_IN_RING_OFFS 1024
#define XENKBD_IN_RING(page) \
	((union xenkbd_in_event *)((char *)(page) + XENKBD_IN_RING_OFFS))
#define XENKBD_IN_RING_REF(page, idx) \
	(XENKBD_IN_RING((page))[(idx) % XENKBD_IN_RING_LEN])

#define XENKBD_OUT_RING_SIZE 1024
#define XENKBD_OUT_RING_LEN (XENKBD_OUT_RING_SIZE / XENKBD_OUT_EVENT_SIZE)
#define XENKBD_OUT_RING_OFFS (XENKBD_IN_RING_OFFS + XENKBD_IN_RING_SIZE)
#define XENKBD_OUT_RING(page) \
	((union xenkbd_out_event *)((char *)(page) + XENKBD_OUT_RING_OFFS))
#define XENKBD_OUT_RING_REF(page, idx) \
	(XENKBD_OUT_RING((page))[(idx) % XENKBD_OUT_RING_LEN])

struct xenkbd_page {
	uint32_t in_cons, in_prod;
	uint32_t out_cons, out_prod;
};

#endif
