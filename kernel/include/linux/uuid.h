

#ifndef _LINUX_UUID_H_
#define _LINUX_UUID_H_

#include <linux/types.h>
#include <linux/string.h>

typedef struct {
	__u8 b[16];
} uuid_le;

typedef struct {
	__u8 b[16];
} uuid_be;

#define UUID_LE(a, b, c, d0, d1, d2, d3, d4, d5, d6, d7)		\
((uuid_le)								\
{{ (a) & 0xff, ((a) >> 8) & 0xff, ((a) >> 16) & 0xff, ((a) >> 24) & 0xff, \
   (b) & 0xff, ((b) >> 8) & 0xff,					\
   (c) & 0xff, ((c) >> 8) & 0xff,					\
   (d0), (d1), (d2), (d3), (d4), (d5), (d6), (d7) }})

#define UUID_BE(a, b, c, d0, d1, d2, d3, d4, d5, d6, d7)		\
((uuid_be)								\
{{ ((a) >> 24) & 0xff, ((a) >> 16) & 0xff, ((a) >> 8) & 0xff, (a) & 0xff, \
   ((b) >> 8) & 0xff, (b) & 0xff,					\
   ((c) >> 8) & 0xff, (c) & 0xff,					\
   (d0), (d1), (d2), (d3), (d4), (d5), (d6), (d7) }})

#define NULL_UUID_LE							\
	UUID_LE(0x00000000, 0x0000, 0x0000, 0x00, 0x00, 0x00, 0x00,	\
		0x00, 0x00, 0x00, 0x00)

#define NULL_UUID_BE							\
	UUID_BE(0x00000000, 0x0000, 0x0000, 0x00, 0x00, 0x00, 0x00,	\
		0x00, 0x00, 0x00, 0x00)

static inline int uuid_le_cmp(const uuid_le u1, const uuid_le u2)
{
	return memcmp(&u1, &u2, sizeof(uuid_le));
}

static inline int uuid_be_cmp(const uuid_be u1, const uuid_be u2)
{
	return memcmp(&u1, &u2, sizeof(uuid_be));
}

extern void uuid_le_gen(uuid_le *u);
extern void uuid_be_gen(uuid_be *u);

#endif
