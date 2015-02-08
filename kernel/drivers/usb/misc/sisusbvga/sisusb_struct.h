

#ifndef _SISUSB_STRUCT_H_
#define _SISUSB_STRUCT_H_

struct SiS_St {
	unsigned char St_ModeID;
	unsigned short St_ModeFlag;
	unsigned char St_StTableIndex;
	unsigned char St_CRT2CRTC;
	unsigned char St_ResInfo;
	unsigned char VB_StTVFlickerIndex;
	unsigned char VB_StTVEdgeIndex;
	unsigned char VB_StTVYFilterIndex;
	unsigned char St_PDC;
};

struct SiS_StandTable {
	unsigned char CRT_COLS;
	unsigned char ROWS;
	unsigned char CHAR_HEIGHT;
	unsigned short CRT_LEN;
	unsigned char SR[4];
	unsigned char MISC;
	unsigned char CRTC[0x19];
	unsigned char ATTR[0x14];
	unsigned char GRC[9];
};

struct SiS_StResInfo_S {
	unsigned short HTotal;
	unsigned short VTotal;
};

struct SiS_Ext {
	unsigned char Ext_ModeID;
	unsigned short Ext_ModeFlag;
	unsigned short Ext_VESAID;
	unsigned char Ext_RESINFO;
	unsigned char VB_ExtTVFlickerIndex;
	unsigned char VB_ExtTVEdgeIndex;
	unsigned char VB_ExtTVYFilterIndex;
	unsigned char VB_ExtTVYFilterIndexROM661;
	unsigned char REFindex;
	char ROMMODEIDX661;
};

struct SiS_Ext2 {
	unsigned short Ext_InfoFlag;
	unsigned char Ext_CRT1CRTC;
	unsigned char Ext_CRTVCLK;
	unsigned char Ext_CRT2CRTC;
	unsigned char Ext_CRT2CRTC_NS;
	unsigned char ModeID;
	unsigned short XRes;
	unsigned short YRes;
	unsigned char Ext_PDC;
	unsigned char Ext_FakeCRT2CRTC;
	unsigned char Ext_FakeCRT2Clk;
};

struct SiS_CRT1Table {
	unsigned char CR[17];
};

struct SiS_VCLKData {
	unsigned char SR2B, SR2C;
	unsigned short CLOCK;
};

struct SiS_ModeResInfo {
	unsigned short HTotal;
	unsigned short VTotal;
	unsigned char XChar;
	unsigned char YChar;
};

struct SiS_Private {
	void *sisusb;

	unsigned long IOAddress;

	unsigned long SiS_P3c4;
	unsigned long SiS_P3d4;
	unsigned long SiS_P3c0;
	unsigned long SiS_P3ce;
	unsigned long SiS_P3c2;
	unsigned long SiS_P3ca;
	unsigned long SiS_P3c6;
	unsigned long SiS_P3c7;
	unsigned long SiS_P3c8;
	unsigned long SiS_P3c9;
	unsigned long SiS_P3cb;
	unsigned long SiS_P3cc;
	unsigned long SiS_P3cd;
	unsigned long SiS_P3da;
	unsigned long SiS_Part1Port;

	unsigned char SiS_MyCR63;
	unsigned short SiS_CRT1Mode;
	unsigned short SiS_ModeType;
	unsigned short SiS_SetFlag;

	const struct SiS_StandTable *SiS_StandTable;
	const struct SiS_St *SiS_SModeIDTable;
	const struct SiS_Ext *SiS_EModeIDTable;
	const struct SiS_Ext2 *SiS_RefIndex;
	const struct SiS_CRT1Table *SiS_CRT1Table;
	const struct SiS_VCLKData *SiS_VCLKData;
	const struct SiS_ModeResInfo *SiS_ModeResInfo;
};

#endif
