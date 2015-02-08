
#ifndef __SOUND_TLV_H
#define __SOUND_TLV_H



#define SNDRV_CTL_TLVT_CONTAINER 0	/* one level down - group of TLVs */
#define SNDRV_CTL_TLVT_DB_SCALE	1       /* dB scale */
#define SNDRV_CTL_TLVT_DB_LINEAR 2	/* linear volume */
#define SNDRV_CTL_TLVT_DB_RANGE 3	/* dB range container */
#define SNDRV_CTL_TLVT_DB_MINMAX 4	/* dB scale with min/max */
#define SNDRV_CTL_TLVT_DB_MINMAX_MUTE 5	/* dB scale with min/max with mute */

#define TLV_DB_SCALE_ITEM(min, step, mute)			\
	SNDRV_CTL_TLVT_DB_SCALE, 2 * sizeof(unsigned int),	\
	(min), ((step) & 0xffff) | ((mute) ? 0x10000 : 0)
#define DECLARE_TLV_DB_SCALE(name, min, step, mute) \
	unsigned int name[] = { TLV_DB_SCALE_ITEM(min, step, mute) }

/* dB scale specified with min/max values instead of step */
#define TLV_DB_MINMAX_ITEM(min_dB, max_dB)			\
	SNDRV_CTL_TLVT_DB_MINMAX, 2 * sizeof(unsigned int),	\
	(min_dB), (max_dB)
#define TLV_DB_MINMAX_MUTE_ITEM(min_dB, max_dB)			\
	SNDRV_CTL_TLVT_DB_MINMAX_MUTE, 2 * sizeof(unsigned int),	\
	(min_dB), (max_dB)
#define DECLARE_TLV_DB_MINMAX(name, min_dB, max_dB) \
	unsigned int name[] = { TLV_DB_MINMAX_ITEM(min_dB, max_dB) }
#define DECLARE_TLV_DB_MINMAX_MUTE(name, min_dB, max_dB) \
	unsigned int name[] = { TLV_DB_MINMAX_MUTE_ITEM(min_dB, max_dB) }

/* linear volume between min_dB and max_dB (.01dB unit) */
#define TLV_DB_LINEAR_ITEM(min_dB, max_dB)		    \
	SNDRV_CTL_TLVT_DB_LINEAR, 2 * sizeof(unsigned int), \
	(min_dB), (max_dB)
#define DECLARE_TLV_DB_LINEAR(name, min_dB, max_dB)	\
	unsigned int name[] = { TLV_DB_LINEAR_ITEM(min_dB, max_dB) }

/* dB range container */
/* Each item is: <min> <max> <TLV> */
/* The below assumes that each item TLV is 4 words like DB_SCALE or LINEAR */
#define TLV_DB_RANGE_HEAD(num)			\
	SNDRV_CTL_TLVT_DB_RANGE, 6 * (num) * sizeof(unsigned int)

#define TLV_DB_GAIN_MUTE	-9999999

#endif /* __SOUND_TLV_H */
