

#ifndef __VIAMODE_H__
#define __VIAMODE_H__

#include "global.h"

struct VPITTable {
	unsigned char Misc;
	unsigned char SR[StdSR];
	unsigned char GR[StdGR];
	unsigned char AR[StdAR];
};

struct VideoModeTable {
	struct crt_mode_table *crtc;
	int mode_array;
};

struct patch_table {
	int table_length;
	struct io_reg *io_reg_table;
};

struct res_map_refresh {
	int hres;
	int vres;
	int pixclock;
	int vmode_refresh;
};

extern int NUM_TOTAL_RES_MAP_REFRESH;
extern int NUM_TOTAL_CEA_MODES;
extern int NUM_TOTAL_CN400_ModeXregs;
extern int NUM_TOTAL_CN700_ModeXregs;
extern int NUM_TOTAL_KM400_ModeXregs;
extern int NUM_TOTAL_CX700_ModeXregs;
extern int NUM_TOTAL_VX855_ModeXregs;
extern int NUM_TOTAL_CLE266_ModeXregs;
extern int NUM_TOTAL_PATCH_MODE;

/********************/
/* Mode Table       */
/********************/

extern struct crt_mode_table CEAM1280x720[];
extern struct crt_mode_table CEAM1920x1080[];
extern struct VideoModeTable CEA_HDMI_Modes[];

extern struct res_map_refresh res_map_refresh_tbl[];
extern struct io_reg CN400_ModeXregs[];
extern struct io_reg CN700_ModeXregs[];
extern struct io_reg KM400_ModeXregs[];
extern struct io_reg CX700_ModeXregs[];
extern struct io_reg VX800_ModeXregs[];
extern struct io_reg VX855_ModeXregs[];
extern struct io_reg CLE266_ModeXregs[];
extern struct io_reg PM1024x768[];
extern struct patch_table res_patch_table[];
extern struct VPITTable VPIT;

struct VideoModeTable *viafb_get_mode(int hres, int vres);
struct VideoModeTable *viafb_get_rb_mode(int hres, int vres);

#endif /* __VIAMODE_H__ */
