

#ifndef _LINUX_HWMON_VID_H
#define _LINUX_HWMON_VID_H

int vid_from_reg(int val, u8 vrm);
u8 vid_which_vrm(void);

static inline int vid_to_reg(int val, u8 vrm)
{
	switch (vrm) {
	case 91:		/* VRM 9.1 */
	case 90:		/* VRM 9.0 */
		return ((val >= 1100) && (val <= 1850) ?
			((18499 - val * 10) / 25 + 5) / 10 : -1);
	default:
		return -1;
	}
}

#endif /* _LINUX_HWMON_VID_H */
