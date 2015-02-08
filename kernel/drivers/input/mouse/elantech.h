

#ifndef _ELANTECH_H
#define _ELANTECH_H

#define ETP_FW_VERSION_QUERY		0x01
#define ETP_CAPABILITIES_QUERY		0x02

#define ETP_REGISTER_READ		0x10
#define ETP_REGISTER_WRITE		0x11

#define ETP_PS2_CUSTOM_COMMAND		0xf8

#define ETP_PS2_COMMAND_TRIES		3
#define ETP_PS2_COMMAND_DELAY		500

#define ETP_READ_BACK_TRIES		5
#define ETP_READ_BACK_DELAY		2000

#define ETP_R10_ABSOLUTE_MODE		0x04
#define ETP_R11_4_BYTE_MODE		0x02

#define ETP_CAP_HAS_ROCKER		0x04

#define ETP_EDGE_FUZZ_V1		32

#define ETP_XMIN_V1			(  0 + ETP_EDGE_FUZZ_V1)
#define ETP_XMAX_V1			(576 - ETP_EDGE_FUZZ_V1)
#define ETP_YMIN_V1			(  0 + ETP_EDGE_FUZZ_V1)
#define ETP_YMAX_V1			(384 - ETP_EDGE_FUZZ_V1)

#define ETP_EDGE_FUZZ_V2		8

#define ETP_XMIN_V2			(   0 + ETP_EDGE_FUZZ_V2)
#define ETP_XMAX_V2			(1152 - ETP_EDGE_FUZZ_V2)
#define ETP_YMIN_V2			(   0 + ETP_EDGE_FUZZ_V2)
#define ETP_YMAX_V2			( 768 - ETP_EDGE_FUZZ_V2)

#define ETP_2FT_FUZZ			4

#define ETP_2FT_XMIN			(  0 + ETP_2FT_FUZZ)
#define ETP_2FT_XMAX			(288 - ETP_2FT_FUZZ)
#define ETP_2FT_YMIN			(  0 + ETP_2FT_FUZZ)
#define ETP_2FT_YMAX			(192 - ETP_2FT_FUZZ)

struct elantech_data {
	unsigned char reg_10;
	unsigned char reg_11;
	unsigned char reg_20;
	unsigned char reg_21;
	unsigned char reg_22;
	unsigned char reg_23;
	unsigned char reg_24;
	unsigned char reg_25;
	unsigned char reg_26;
	unsigned char debug;
	unsigned char capabilities;
	unsigned char paritycheck;
	unsigned char jumpy_cursor;
	unsigned char hw_version;
	unsigned int  fw_version;
	unsigned char parity[256];
};

#ifdef CONFIG_MOUSE_PS2_ELANTECH
int elantech_detect(struct psmouse *psmouse, bool set_properties);
int elantech_init(struct psmouse *psmouse);
#else
static inline int elantech_detect(struct psmouse *psmouse, bool set_properties)
{
	return -ENOSYS;
}
static inline int elantech_init(struct psmouse *psmouse)
{
	return -ENOSYS;
}
#endif /* CONFIG_MOUSE_PS2_ELANTECH */

#endif
