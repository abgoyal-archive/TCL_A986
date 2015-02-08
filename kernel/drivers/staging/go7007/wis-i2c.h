

/* Temporary I2C IDs -- these need to be replaced with real registered IDs */
#define	I2C_DRIVERID_WIS_SAA7115	0xf0f0
#define	I2C_DRIVERID_WIS_UDA1342	0xf0f1
#define	I2C_DRIVERID_WIS_SONY_TUNER	0xf0f2
#define	I2C_DRIVERID_WIS_TW9903		0xf0f3
#define	I2C_DRIVERID_WIS_SAA7113	0xf0f4
#define	I2C_DRIVERID_WIS_OV7640		0xf0f5
#define	I2C_DRIVERID_WIS_TW2804		0xf0f6
#define	I2C_DRIVERID_S2250		0xf0f7

/* Flag to indicate that the client needs to be accessed with SCCB semantics */
#define	I2C_CLIENT_SCCB			0x10

/* Definitions for new video decoder commands */

struct video_decoder_resolution {
	unsigned int width;
	unsigned int height;
};

#define	DECODER_SET_RESOLUTION	_IOW('d', 200, struct video_decoder_resolution)
#define	DECODER_SET_CHANNEL	_IOW('d', 201, int)

/* Sony tuner types */

#define TUNER_SONY_BTF_PG472Z		200
#define TUNER_SONY_BTF_PK467Z		201
#define TUNER_SONY_BTF_PB463Z		202
