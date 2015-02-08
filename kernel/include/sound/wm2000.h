

#ifndef __LINUX_SND_WM2000_H
#define __LINUX_SND_WM2000_H

struct wm2000_platform_data {
	/** Filename for system-specific image to download to device. */
	const char *download_file;

	/** Divide MCLK by 2 for system clock? */
	unsigned int mclkdiv2:1;

	/** Disable speech clarity enhancement, for use when an
	 * external algorithm is used. */
	unsigned int speech_enh_disable:1;
};

#endif
