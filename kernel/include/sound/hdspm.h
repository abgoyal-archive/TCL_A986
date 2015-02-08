
#ifndef __SOUND_HDSPM_H
#define __SOUND_HDSPM_H

/* Maximum channels is 64 even on 56Mode you have 64playbacks to matrix */
#define HDSPM_MAX_CHANNELS      64

/* -------------------- IOCTL Peak/RMS Meters -------------------- */


struct hdspm_peak_rms {

	unsigned int level_offset[1024];

	unsigned int input_peak[64];
	unsigned int playback_peak[64];
	unsigned int output_peak[64];
	unsigned int xxx_peak[64];	/* not used */

	unsigned int reserved[256];	/* not used */

	unsigned int input_rms_l[64];
	unsigned int playback_rms_l[64];
	unsigned int output_rms_l[64];
	unsigned int xxx_rms_l[64];	/* not used */

	unsigned int input_rms_h[64];
	unsigned int playback_rms_h[64];
	unsigned int output_rms_h[64];
	unsigned int xxx_rms_h[64];	/* not used */
};

struct hdspm_peak_rms_ioctl {
	struct hdspm_peak_rms *peak;
};

/* use indirect access due to the limit of ioctl bit size */
#define SNDRV_HDSPM_IOCTL_GET_PEAK_RMS \
	_IOR('H', 0x40, struct hdspm_peak_rms_ioctl)

/* ------------ CONFIG block IOCTL ---------------------- */

struct hdspm_config_info {
	unsigned char pref_sync_ref;
	unsigned char wordclock_sync_check;
	unsigned char madi_sync_check;
	unsigned int system_sample_rate;
	unsigned int autosync_sample_rate;
	unsigned char system_clock_mode;
	unsigned char clock_source;
	unsigned char autosync_ref;
	unsigned char line_out;
	unsigned int passthru;
	unsigned int analog_out;
};

#define SNDRV_HDSPM_IOCTL_GET_CONFIG_INFO \
	_IOR('H', 0x41, struct hdspm_config_info)


/* get Soundcard Version */

struct hdspm_version {
	unsigned short firmware_rev;
};

#define SNDRV_HDSPM_IOCTL_GET_VERSION _IOR('H', 0x43, struct hdspm_version)


/* ------------- get Matrix Mixer IOCTL --------------- */


/* organisation is 64 channelfader in a continous memory block */

#define HDSPM_MIXER_CHANNELS HDSPM_MAX_CHANNELS

struct hdspm_channelfader {
	unsigned int in[HDSPM_MIXER_CHANNELS];
	unsigned int pb[HDSPM_MIXER_CHANNELS];
};

struct hdspm_mixer {
	struct hdspm_channelfader ch[HDSPM_MIXER_CHANNELS];
};

struct hdspm_mixer_ioctl {
	struct hdspm_mixer *mixer;
};

/* use indirect access due to the limit of ioctl bit size */
#define SNDRV_HDSPM_IOCTL_GET_MIXER _IOR('H', 0x44, struct hdspm_mixer_ioctl)

/* typedefs for compatibility to user-space */
typedef struct hdspm_peak_rms hdspm_peak_rms_t;
typedef struct hdspm_config_info hdspm_config_info_t;
typedef struct hdspm_version hdspm_version_t;
typedef struct hdspm_channelfader snd_hdspm_channelfader_t;
typedef struct hdspm_mixer hdspm_mixer_t;

#endif				/* __SOUND_HDSPM_H */
