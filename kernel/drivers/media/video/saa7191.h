

#ifndef _SAA7191_H_
#define _SAA7191_H_

/* Philips SAA7191 DMSD I2C bus address */
#define SAA7191_ADDR		0x8a

/* Register subaddresses. */
#define SAA7191_REG_IDEL	0x00
#define SAA7191_REG_HSYB	0x01
#define SAA7191_REG_HSYS	0x02
#define SAA7191_REG_HCLB	0x03
#define SAA7191_REG_HCLS	0x04
#define SAA7191_REG_HPHI	0x05
#define SAA7191_REG_LUMA	0x06
#define SAA7191_REG_HUEC	0x07
#define SAA7191_REG_CKTQ	0x08 /* bits 3-7 */
#define SAA7191_REG_CKTS	0x09 /* bits 3-7 */
#define SAA7191_REG_PLSE	0x0a
#define SAA7191_REG_SESE	0x0b
#define SAA7191_REG_GAIN	0x0c
#define SAA7191_REG_STDC	0x0d
#define SAA7191_REG_IOCK	0x0e
#define SAA7191_REG_CTL3	0x0f
#define SAA7191_REG_CTL4	0x10
#define SAA7191_REG_CHCV	0x11
#define SAA7191_REG_HS6B	0x14
#define SAA7191_REG_HS6S	0x15
#define SAA7191_REG_HC6B	0x16
#define SAA7191_REG_HC6S	0x17
#define SAA7191_REG_HP6I	0x18
#define SAA7191_REG_STATUS	0xff	/* not really a subaddress */

/* Status Register definitions */
#define SAA7191_STATUS_CODE	0x01	/* color detected flag */
#define SAA7191_STATUS_FIDT	0x20	/* signal type 50/60 Hz */
#define SAA7191_STATUS_HLCK	0x40	/* PLL unlocked(1)/locked(0) */
#define SAA7191_STATUS_STTC	0x80	/* tv/vtr time constant */

/* Luminance Control Register definitions */
#define SAA7191_LUMA_BYPS	0x80
/* pre-filter (only when chrominance trap is active) */
#define SAA7191_LUMA_PREF	0x40
#define SAA7191_LUMA_BPSS_MASK	0x30
#define SAA7191_LUMA_BPSS_SHIFT	4
#define SAA7191_LUMA_BPSS_3	0x30
#define SAA7191_LUMA_BPSS_2	0x20
#define SAA7191_LUMA_BPSS_1	0x10
#define SAA7191_LUMA_BPSS_0	0x00
#define SAA7191_LUMA_CORI_MASK	0x0c
#define SAA7191_LUMA_CORI_SHIFT	2
#define SAA7191_LUMA_CORI_3	0x0c
#define SAA7191_LUMA_CORI_2	0x08
#define SAA7191_LUMA_CORI_1	0x04
#define SAA7191_LUMA_CORI_0	0x00
#define SAA7191_LUMA_APER_MASK	0x03
#define SAA7191_LUMA_APER_SHIFT	0
#define SAA7191_LUMA_APER_3	0x03
#define SAA7191_LUMA_APER_2	0x02
#define SAA7191_LUMA_APER_1	0x01
#define SAA7191_LUMA_APER_0	0x00

/* Chrominance Gain Control Settings Register definitions */
/* colour on: 0=automatic colour-killer enabled, 1=forced colour on */
#define SAA7191_GAIN_COLO	0x80
#define SAA7191_GAIN_LFIS_MASK	0x60
#define SAA7191_GAIN_LFIS_SHIFT	5
#define SAA7191_GAIN_LFIS_3	0x60
#define SAA7191_GAIN_LFIS_2	0x40
#define SAA7191_GAIN_LFIS_1	0x20
#define SAA7191_GAIN_LFIS_0	0x00

/* Standard/Mode Control Register definitions */
#define SAA7191_STDC_VTRC	0x80
#define SAA7191_STDC_NFEN	0x08
/* HREF generation: 0=like SAA7191, 1=HREF is 8xLLC2 clocks earlier */
#define SAA7191_STDC_HRMV	0x04
#define SAA7191_STDC_GPSW0	0x02
/* SECAM mode bit: 0=other standards, 1=SECAM */
#define SAA7191_STDC_SECS	0x01

/* I/O and Clock Control Register definitions */
#define SAA7191_IOCK_HPLL	0x80
/* colour-difference output enable (outputs UV0-UV7) */
#define SAA7191_IOCK_OEDC	0x40
/* H-sync output enable */
#define SAA7191_IOCK_OEHS	0x20
/* V-sync output enable */
#define SAA7191_IOCK_OEVS	0x10
/* luminance output enable (outputs Y0-Y7) */
#define SAA7191_IOCK_OEDY	0x08
#define SAA7191_IOCK_CHRS	0x04
#define SAA7191_IOCK_GPSW2	0x02
/* general purpose switch 1 */
/* VINO-specific: 0=always, 1=not used!*/
#define SAA7191_IOCK_GPSW1	0x01

/* Miscellaneous Control #1 Register definitions */
/* automatic field detection (50/60Hz standard) */
#define SAA7191_CTL3_AUFD	0x80
#define SAA7191_CTL3_FSEL	0x40
/* SECAM cross-colour reduction enable */
#define SAA7191_CTL3_SXCR	0x20
/* sync and clamping pulse enable (HCL and HSY outputs) */
#define SAA7191_CTL3_SCEN	0x10
/* output format: 0=4:1:1, 1=4:2:2 (4:2:2 for VINO) */
#define SAA7191_CTL3_OFTS	0x08
#define SAA7191_CTL3_YDEL_MASK	0x07
#define SAA7191_CTL3_YDEL_SHIFT	0
#define SAA7191_CTL3_YDEL2	0x04
#define SAA7191_CTL3_YDEL1	0x02
#define SAA7191_CTL3_YDEL0	0x01

/* Miscellaneous Control #2 Register definitions */
#define SAA7191_CTL4_HRFS	0x04
#define SAA7191_CTL4_VNOI_MASK	0x03
#define SAA7191_CTL4_VNOI_SHIFT	0
#define SAA7191_CTL4_VNOI_3	0x03
#define SAA7191_CTL4_VNOI_2	0x02
#define SAA7191_CTL4_VNOI_1	0x01
#define SAA7191_CTL4_VNOI_0	0x00

#define SAA7191_CHCV_NTSC	0x2c
#define SAA7191_CHCV_PAL	0x59

/* Driver interface definitions */
#define SAA7191_INPUT_COMPOSITE	0
#define SAA7191_INPUT_SVIDEO	1

#define SAA7191_NORM_PAL	1
#define SAA7191_NORM_NTSC	2
#define SAA7191_NORM_SECAM	3

struct saa7191_status {
	/* 0=no signal, 1=signal detected */
	int signal;
	/* 0=50hz (pal) signal, 1=60hz (ntsc) signal */
	int signal_60hz;
	/* 0=no color detected, 1=color detected */
	int color;

	/* current SAA7191_INPUT_ */
	int input;
	/* current SAA7191_NORM_ */
	int norm;
};

#define SAA7191_BANDPASS_MIN		0x00
#define SAA7191_BANDPASS_MAX		0x03
#define SAA7191_BANDPASS_DEFAULT	0x00

#define SAA7191_BANDPASS_WEIGHT_MIN	0x00
#define SAA7191_BANDPASS_WEIGHT_MAX	0x03
#define SAA7191_BANDPASS_WEIGHT_DEFAULT	0x01

#define SAA7191_CORING_MIN		0x00
#define SAA7191_CORING_MAX		0x03
#define SAA7191_CORING_DEFAULT		0x00

#define SAA7191_HUE_MIN			0x00
#define SAA7191_HUE_MAX			0xff
#define SAA7191_HUE_DEFAULT		0x80

#define SAA7191_VTRC_MIN		0x00
#define SAA7191_VTRC_MAX		0x01
#define SAA7191_VTRC_DEFAULT		0x00

#define SAA7191_FORCE_COLOUR_MIN	0x00
#define SAA7191_FORCE_COLOUR_MAX	0x01
#define SAA7191_FORCE_COLOUR_DEFAULT	0x00

#define SAA7191_CHROMA_GAIN_MIN		0x00
#define SAA7191_CHROMA_GAIN_MAX		0x03
#define SAA7191_CHROMA_GAIN_DEFAULT	0x00

#define SAA7191_LUMA_DELAY_MIN		-0x04
#define SAA7191_LUMA_DELAY_MAX		0x03
#define SAA7191_LUMA_DELAY_DEFAULT	0x01

#define SAA7191_VNR_MIN			0x00
#define SAA7191_VNR_MAX			0x03
#define SAA7191_VNR_DEFAULT		0x00

#define SAA7191_CONTROL_BANDPASS	(V4L2_CID_PRIVATE_BASE + 0)
#define SAA7191_CONTROL_BANDPASS_WEIGHT	(V4L2_CID_PRIVATE_BASE + 1)
#define SAA7191_CONTROL_CORING		(V4L2_CID_PRIVATE_BASE + 2)
#define SAA7191_CONTROL_FORCE_COLOUR	(V4L2_CID_PRIVATE_BASE + 3)
#define SAA7191_CONTROL_CHROMA_GAIN	(V4L2_CID_PRIVATE_BASE + 4)
#define SAA7191_CONTROL_VTRC		(V4L2_CID_PRIVATE_BASE + 5)
#define SAA7191_CONTROL_LUMA_DELAY	(V4L2_CID_PRIVATE_BASE + 6)
#define SAA7191_CONTROL_VNR		(V4L2_CID_PRIVATE_BASE + 7)

#define	DECODER_SAA7191_GET_STATUS	_IOR('d', 195, struct saa7191_status)
#define	DECODER_SAA7191_SET_NORM	_IOW('d', 196, int)

#endif
