
#ifndef _KD_IMGSENSOR_H
#define _KD_IMGSENSOR_H

#include <linux/ioctl.h>


#define IMGSENSORMAGIC 'i'
//IOCTRL(inode * ,file * ,cmd ,arg )
//S means "set through a ptr"
//T means "tell by a arg value"
//G means "get by a ptr"
//Q means "get by return a value"
//X means "switch G and S atomically"
//H means "switch T and Q atomically"



//sensorOpen
//This command will TBD
#define KDIMGSENSORIOC_T_OPEN            _IO(IMGSENSORMAGIC,0)
//sensorGetInfo
//This command will TBD
#define KDIMGSENSORIOC_X_GETINFO            _IOWR(IMGSENSORMAGIC,5,ACDK_SENSOR_GETINFO_STRUCT)
//sensorGetResolution
//This command will TBD
#define KDIMGSENSORIOC_X_GETRESOLUTION      _IOWR(IMGSENSORMAGIC,10,ACDK_SENSOR_RESOLUTION_INFO_STRUCT)
//sensorFeatureControl
//This command will TBD
#define KDIMGSENSORIOC_X_FEATURECONCTROL    _IOWR(IMGSENSORMAGIC,15,ACDK_SENSOR_FEATURECONTROL_STRUCT)
//sensorControl
//This command will TBD
#define KDIMGSENSORIOC_X_CONTROL            _IOWR(IMGSENSORMAGIC,20,ACDK_SENSOR_CONTROL_STRUCT)
//sensorClose
//This command will TBD
#define KDIMGSENSORIOC_T_CLOSE            _IO(IMGSENSORMAGIC,25)
//set sensor driver
#define KDIMGSENSORIOC_X_SET_DRIVER         _IOWR(IMGSENSORMAGIC,40,u32)
//sensorSearch 
#define KDIMGSENSORIOC_T_CHECK_IS_ALIVE     _IO(IMGSENSORMAGIC, 30) 

/* SENSOR CHIP VERSION */
#define MC501CB_SENSOR_ID                       0x0062
#define MC501CC_SENSOR_ID                       0x0074
#define MC501CA_SENSOR_ID                       0x0011

#define MT9D011_SENSOR_ID                       0x1511
#define MT9D111_SENSOR_ID                       0x1511
#define MT9D112_SENSOR_ID                       0x1580
#define MT9M011_SENSOR_ID                       0x1433
#define MT9M111_SENSOR_ID                       0x143A
#define MT9M112_SENSOR_ID                       0x148C
#define MT9M113_SENSOR_ID                       0x2480
#define MT9P012_SENSOR_ID						0x2800
#define MT9P012_SENSOR_ID_REV7					0x2801
#define MT9T012_SENSOR_ID                       0x1600
#define MT9T013_SENSOR_ID                       0x2600
#define MT9T113_SENSOR_ID                       0x4680
#define MT9V112_SENSOR_ID                       0x1229
#define MT9DX11_SENSOR_ID                       0x1519
#define MT9D113_SENSOR_ID                       0x2580
#define MT9T112_SENSOR_ID                       0x2682
#define MT9D115_SENSOR_ID                       0x2580
#define MT9V113_SENSOR_ID                       0x2280
#define MT9V114_SENSOR_ID                       0x2283
#define MT9P015_SENSOR_ID                       0x2803


#define NOON200PC11_SENSOR_ID                   0x0013
#define NOON200PC20_SENSOR_ID                   0x0063
#define NOON200PC40_SENSOR_ID                   0x0063
#define NOON200PC51_SENSOR_ID                   0x006C
#define NOON130PC51_SENSOR_ID                   0x0076

#define OV2630_SENSOR_ID                        0x2633
#define OV2640_SENSOR_ID                        0x2642
#define OV2650_SENSOR_ID                        0x2652
#define OV3640_SENSOR_ID                        0x364C
#define OV6680_SENSOR_ID                        0x6681
#define OV7660_SENSOR_ID                        0x7660
#define OV7670_SENSOR_ID                        0x7673
#define OV7680_SENSOR_ID                        0x7680
#define OV7690_SENSOR_ID                        0x7691
#define OV9650_SENSOR_ID                        0x9652
#define OV9655_SENSOR_ID                        0x9657
#define OV9660_SENSOR_ID                        0x9663
#define OV3647_SENSOR_ID                        0x364A
#define OV2655_SENSOR_ID					    0x2656
#define OV2650_SENSOR_ID_1                      0x2651
#define OV2650_SENSOR_ID_2                      0x2652
#define OV2650_SENSOR_ID_3			            0x2655
#define OV5642_SENSOR_ID            			0x5642
#define OV5650_SENSOR_ID                        0x5651
#define OV9665_SENSOR_ID                        0x9663
#define OV5630_SENSOR_ID                        0x5634
#define OV7675_SENSOR_ID                        0x7673
#define OV5647_SENSOR_ID                        0x5647


#define PO6030K_SENSOR_ID                       0x0060
#define PO4010K_SENSOR_ID                       0x0040

#define SID020A_SENSOR_ID                       0x12B4
#define SIV100B_SENSOR_ID                       0x0C11
#define SIV100A_SENSOR_ID                       0x0C10
#define SIV120A_SENSOR_ID                       0x1210
#define SIV120B_SENSOR_ID                       0x0012
#define SIM101B_SENSOR_ID                       0x09A0
#define SIM120C_SENSOR_ID                       0x0012
#define SID130B_SENSOR_ID                       0x001b
#define SIC110A_SENSOR_ID                       0x000D
#define SIV120B_SENSOR_ID                       0x0012

#define S5KA3DFX_SENSOR_ID                      0x00AB
#define S5K4B2FX_SENSOR_ID                      0x5080
#define S5K3AAEA_SENSOR_ID                      0x07AC
#define S5K3BAFB_SENSOR_ID                      0x7070
#define S5K53BEX_SENSOR_ID                      0x45A8
#define S5K53BEB_SENSOR_ID                      0x87A8
#define S5K83AFX_SENSOR_ID                      0x01C4
#define S5K5BAFX_SENSOR_ID                      0x05BA
#define S5K3E2FX_SENSOR_ID                      0x3E2F
#define S5K5CAGX_SENSOR_ID                      0x05ca

#define PAS105_SENSOR_ID                        0x0065
#define PAS302_SENSOR_ID                        0x0064
#define PAS5101_SENSOR_ID                       0x0067

#define ET8EE6_SENSOR_ID                        0x0034
#define ET8EF2_SENSOR_ID                        0x1048

#define OM6802_SENSOR_ID                        0x1705

#define HV7131_SENSOR_ID                        0x0042

#define RJ53S1BA0C_SENSOR_ID                    0x0129

#define HI251_SENSOR_ID                         0x0084
#define HIVICF_SENSOR_ID                        0x0081
#define HI253_SENSOR_ID                         0x0092
#define HI342_SENSOR_ID                         0x0093
#define HI704_SENSOR_ID                         0x0096

#define IMX058_SENSOR_ID                        0x0058
#define IMX073_SENSOR_ID                        0x0046

#define GC0309_SENSOR_ID                        0x00a0
#define GT2005_SENSOR_ID                        0x5138

#define HM3451_SENSOR_ID                        0x3451

/* CAMERA DRIVER NAME */
#define CAMERA_HW_DEVNAME            	"kd_camera_hw"

/* SENSOR DEVICE DRIVER NAME */
#define SENSOR_DRVNAME_MT9P012_RAW  	"mt9p012"
#define SENSOR_DRVNAME_MT9P015_RAW  	"mt9p015"
#define SENSOR_DRVNAME_MT9T112_YUV    	"mt9t112yuv"
#define SENSOR_DRVNAME_MT9T113_YUV    	"mt9t113yuv"
#define SENSOR_DRVNAME_MT9D113_YUV   	"mt9d113yuv"
#define SENSOR_DRVNAME_MT9D115_YUV   	"mt9d115yuv"
#define SENSOR_DRVNAME_MT9V113_YUV    	"mt9v113yuv"
#define SENSOR_DRVNAME_MT9V114_YUV    	"mt9v114"
#define SENSOR_DRVNAME_OV5642_RAW   	"ov5642raw"
#define SENSOR_DRVNAME_OV5642_MIPI_YUV  "ov5642mipiyuv"
#define SENSOR_DRVNAME_OV5647_RAW   	"ov5647"
#define SENSOR_DRVNAME_OV5650_RAW   	"ov5650raw"
#define SENSOR_DRVNAME_OV3640_RAW   	"ov3640"
#define SENSOR_DRVNAME_OV3640_YUV    	"ov3640yuv"
#define SENSOR_DRVNAME_OV3647_RAW	 	"ov3647raw"
#define SENSOR_DRVNAME_OV2650_RAW   	"ov265x"
#define SENSOR_DRVNAME_OV2655_YUV   	"ov2655yuv"
#define SENSOR_DRVNAME_OV7675_YUV   	"ov7675yuv"
#define SENSOR_DRVNAME_OV7690_YUV    	"ov7690yuv"
#define SENSOR_DRVNAME_IMX073_MIPI_RAW  "imx073mipiraw"
#define SENSOR_DRVNAME_S5K5CAGX_YUV     "s5k5cagxyuv"
#define SENSOR_DRVNAME_SID130B_YUV    	"sid130byuv"
#define SENSOR_DRVNAME_SIV120B_YUV    	"siv120byuv"
#define SENSOR_DRVNAME_HI342_YUV    	"hi342yuv"
#define SENSOR_DRVNAME_HI342_RAW    	"hi342"
#define SENSOR_DRVNAME_HI253_YUV    	"hi253yuv"
#define SENSOR_DRVNAME_HI704_YUV    	"hi704yuv"
#define SENSOR_DRVNAME_HM3451_RAW    	"hm3451"
#define SENSOR_DRVNAME_GT2005_YUV    	"gt2005yuv"
#define SENSOR_DRVNAME_GC0309_YUV   	"gc0309yuv"


void KD_IMGSENSOR_PROFILE_INIT(void); 
void KD_IMGSENSOR_PROFILE(char *tag); 

#define mDELAY(ms)     mdelay(ms) 
#define uDELAY(us)       udelay(us) 
#endif //_KD_IMGSENSOR_H


