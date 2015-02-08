//#include <windows.h>
//#include <memory.h>
//#include <nkintr.h>
//#include <ceddk.h>
//#include <ceddk_exp.h>

//#include "kal_release.h"
//#include "i2c_exp.h"
//#include "gpio_exp.h"
//#include "msdk_exp.h"
//#include "msdk_sensor_exp.h"
//#include "msdk_isp_exp.h"
//#include "base_regs.h"
//#include "Sensor.h"
//#include "camera_sensor_para.h"
//#include "CameraCustomized.h"

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "sid130byuv_Sensor.h"
#include "sid130byuv_Camera_Sensor_para.h"
#include "sid130byuv_CameraCustomized.h"

#define SID130BYUV_DEBUG
#ifdef SID130BYUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif
#if 0

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define SID130B_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,SID130B_WRITE_ID)
kal_uint16 SID130B_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,SID130B_WRITE_ID);
    return get_byte;
}
#endif
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
kal_uint16 SID130B_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 2,SID130B_WRITE_ID);

}
kal_uint16 SID130B_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd = { (char)(addr & 0xFF) };
	iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte,1,SID130B_WRITE_ID);
#ifdef OV5647_DRIVER_TRACE
	//SENSORDB("OV5647_read_cmos_sensor, addr:%x;get_byte:%x \n",addr,get_byte);
#endif		
    return get_byte;
}


#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT

#define DRV_ISP_YUV_BURST_MODE_SUPPORT


/* Global Valuable */
#if (defined(DRV_ISP_YUV_BURST_MODE_SUPPORT))
#define SID130B_SOURCE_CLK_Pre      52000000
#define SID130B_SOURCE_CLK_Cap      52000000
#else
#define SID130B_SOURCE_CLK_Pre      48000000//48000000
#define SID130B_SOURCE_CLK_Cap      69333333//69333333
#endif
#ifdef SID130B_MT6253_EVB
#define SID130B_PCLK_Pre        (41600000)
#define SID130B_PCLK_Cap        (41600000)
#else						
#define SID130B_PCLK_Pre        (SID130B_SOURCE_CLK_Pre/ SID130B_mclk_div_Pre)
#define SID130B_PCLK_Cap        (SID130B_SOURCE_CLK_Cap/ SID130B_mclk_div_Cap)
#endif

#define SID130B_DEBUG_ANDROID

static kal_uint32 SID130B_zoom_factor = 0; 
static kal_bool SID130B_g_bNightMode = FALSE;
static kal_uint8 SID130B_g_iBanding = AE_FLICKER_MODE_50HZ;
static kal_uint8 SID130B_HVMirror = IMAGE_NORMAL;
static kal_uint16 SID130B_PV_Dummy_Pixel = 0x1c; // for user customization//0x1c
static kal_uint16 SID130B_PV_Hblank = 0x1c;       //for calculating shutter step
static kal_uint16 SID130B_PV_Sutter_Step = 0x96;       //Back up for mode change switch factor
static kal_uint16 SID130B_outdoor_condition = 0x63;       //Back up for outdoor condition
static kal_bool SID130B_Video_Mode = FALSE;
static kal_uint8 SID130B_mclk_div_Pre = 1;//11
static kal_uint8 SID130B_mclk_div_Cap = 1;//11
static kal_bool SID130B_Mode_change = FALSE;
static kal_uint16 SID130B_Min_Fps_Normal; 
static kal_uint16 SID130B_Min_Fps_Night; 
static kal_uint16 SID130B_Min_Fps_Video = 30 * SID130B_FRAME_RATE_UNIT;
static kal_uint8 SID130B_Control = 0x00;

/* MAX/MIN Explosure Lines Used By AE Algorithm */
// must be defined but not referenced by YUV driver
kal_uint16 SID130B_MAX_EXPOSURE_LINES = 1000;  
kal_uint8  SID130B_MIN_EXPOSURE_LINES = 1; 


#ifdef SID130B_DEBUG_ANDROID
// for driver current
kal_uint8 pll=0xa2;
kal_uint8 pllset=0x03;//0x30//03//10
kal_uint8 bb_driver_current=1;//33
kal_uint8 sensor_current = 0x0f;//8//0c
#endif

//SENSOR_REG_STRUCT SID130BSensorCCT[FACTORY_END_ADDR]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
//SENSOR_REG_STRUCT SID130BSensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
//	camera_para.SENSOR.cct	SensorCCT	=> SensorCCT
//	camera_para.SENSOR.reg	SensorReg
MSDK_SENSOR_CONFIG_STRUCT SID130BSensorConfigData;


static void SID130B_set_page(kal_uint8 iPage)
{    
    SID130B_write_cmos_sensor(0x00,iPage);
}

static void SID130B_set_outdoor_fix_gain_conditon(kal_uint8 para)
{    
	SID130B_set_page(0x02); 
	SID130B_write_cmos_sensor(0x27,para);
	SID130B_write_cmos_sensor(0x28,para);
}


   /* SID130B_Initial_Setting */


__inline static kal_uint32 SID130B_half_adjust(kal_uint32 dividend, kal_uint32 divisor)
{
  return (dividend * 2 + divisor) / (divisor * 2); /* that is [dividend / divisor + 0.5]*/
}
   
   
   /*************************************************************************
   * FUNCTION
   *   SID130B_PV_Set_Shutter_Step
   *
   * DESCRIPTION
   *   This function is to calculate & set shutter step register .
   *
   *************************************************************************/
   static void SID130B_PV_Set_Shutter_Step(void)
   {	   
	   const kal_uint8 banding = SID130B_g_iBanding == AE_FLICKER_MODE_50HZ ? SID130B_NUM_50HZ : SID130B_NUM_60HZ;
	   //const kal_uint16 shutter_step = SID130B_half_adjust(SID130B_PCLK / 2, (SID130B_PV_Hblank + SID130B_PV_PERIOD_PIXEL_NUMS) * banding);
	   const kal_uint16 shutter_step = SID130B_PCLK_Pre/ 2 / (SID130B_PV_Hblank + SID130B_PV_PERIOD_PIXEL_NUMS) / banding;
	   //kal_uint16 shutter_step = SID130B_PCLK_Pre/ 2 / (SID130B_PV_Hblank + SID130B_PV_PERIOD_PIXEL_NUMS) / banding;
   
	   ASSERT(shutter_step <= 0xFF);  //haiyong modify
	   //if(shutter_step > 0xff)
		   //shutter_step=0xfe;
	   /* Block 1:0x34	shutter step*/
	   SID130B_set_page(1);
	   SID130B_write_cmos_sensor(0x34,shutter_step);
	   
	   //ASSERT(shutter_step>2);
	   //SID130B_set_page(2);
	   
	   //SID130B_write_cmos_sensor(0x27,shutter_step-2);   
	   //SID130B_write_cmos_sensor(0x28,shutter_step+2);
	   
	   SID130B_PV_Sutter_Step = shutter_step;
   }/* SID130B_PV_Set_Shutter_Step */
   
   
   /*************************************************************************
   * FUNCTION
   *   SID130B_Set_Frame_Count
   *
   * DESCRIPTION
   *   This function is to set frame count register .
   *
   *************************************************************************/
   static void SID130B_Set_Frame_Count(void)
   {	
	   kal_uint16 Frame_Count,min_fps = 100;
	   kal_uint8 banding = SID130B_g_iBanding == AE_FLICKER_MODE_50HZ ? SID130B_NUM_50HZ : SID130B_NUM_60HZ;
   
	   if(SID130B_Min_Fps_Night > SID130B_MAX_CAMERA_FPS)
		   SID130B_Min_Fps_Night = SID130B_MAX_CAMERA_FPS;
   
	   if(SID130B_Min_Fps_Normal > SID130B_MAX_CAMERA_FPS)
		   SID130B_Min_Fps_Normal = SID130B_MAX_CAMERA_FPS;
	   if(  SID130B_Video_Mode == TRUE)
	       min_fps = SID130B_Min_Fps_Video;
	   else
	       min_fps = SID130B_g_bNightMode ? SID130B_Min_Fps_Night : SID130B_Min_Fps_Normal;
	   Frame_Count = banding * SID130B_FRAME_RATE_UNIT / min_fps;
	   /*Block 01: 0x11  Max shutter step,for Min frame rate */
	   SID130B_set_page(1);
	   SID130B_write_cmos_sensor(0x11,Frame_Count&0xFF);	
   }/* SID130B_Set_Frame_Count */



static void SID130B_Config_PV_Blank(kal_uint16 hblank,kal_uint16 vblank)
{    
	/********************************************	
	*   preview mode 0x24 - 0x27 
	*   page 0
	*	 0x24  [7:4]:HBANK[11:8]; 0x20	[3:0]:VBANK[11:8]
	*  0x25 HBANK[7:0]
	*  0x27 VBANK[7:0]
	*  page 1
	*  0x34	shutter step
	********************************************/
    ASSERT(hblank <= SID130B_BLANK_REGISTER_LIMITATION && vblank <= SID130B_BLANK_REGISTER_LIMITATION);
    SID130B_set_page(0);
    SID130B_write_cmos_sensor(0x24,((hblank>>4)&0xF0)|((vblank>>8)&0x0F));
    SID130B_write_cmos_sensor(0x25,hblank & 0xFF);
    SID130B_write_cmos_sensor(0x27,vblank & 0xFF);
    SID130B_PV_Set_Shutter_Step();
}   /* SID130B_Config_PV_Blank */



static void SID130B_Config_Cap_Blank(kal_uint16 hblank,kal_uint16 vblank)
{    
	kal_uint16 P2CFactor;
	kal_uint32 CapOutdoor;
	const kal_uint8 banding = SID130B_g_iBanding == AE_FLICKER_MODE_50HZ ? SID130B_NUM_50HZ : SID130B_NUM_60HZ;
	const kal_uint16 cap_shutter_step = SID130B_half_adjust(SID130B_PCLK_Cap/ 2, (hblank + SID130B_CAP_PERIOD_PIXEL_NUMS) * banding);
	
	/********************************************
	  *   still mode 0x20 - 0x23 *
	  * 	page 0
	  * 	0x20  [7:4]:HBANK[11:8]; 0x20  [3:0]:VBANK[11:8]
	  *  0x21 HBANK[7:0]
	  *  0x23 VBANK[7:0]
	  *  page 1
	  *  0x35  shutter step
	  *  0x36  preview to still adjust factor
	  ********************************************/
    ASSERT(hblank <= SID130B_BLANK_REGISTER_LIMITATION && vblank <= SID130B_BLANK_REGISTER_LIMITATION);
    SID130B_set_page(0);
    SID130B_write_cmos_sensor(0x20,((hblank>>4)&0xF0)|((vblank>>8)&0x0F));
    SID130B_write_cmos_sensor(0x21,hblank & 0xFF);
    SID130B_write_cmos_sensor(0x23,vblank & 0xFF);

	ASSERT(cap_shutter_step <= 0xFF);    
	/* Block 1:0x35  shutter step*/
	SID130B_set_page(1);
	SID130B_write_cmos_sensor(0x35,cap_shutter_step);
	
	P2CFactor  = cap_shutter_step * 64 / SID130B_PV_Sutter_Step;
	SID130B_write_cmos_sensor(0x36,P2CFactor & 0xFF);  
	CapOutdoor = SID130B_outdoor_condition * cap_shutter_step / SID130B_PV_Sutter_Step;
	SID130B_set_outdoor_fix_gain_conditon(CapOutdoor);
}   /* SID130B_Config_Cap_Blank */




static void SID130B_cal_fps(void)
{
    kal_int16 Line_length,Dummy_Line;
    kal_uint16 max_fps = 300;

    Line_length = SID130B_PV_Dummy_Pixel + SID130B_PV_PERIOD_PIXEL_NUMS; 

	{
		kal_uint32 clk = SID130B_PCLK_Pre;
	//	kal_wap_trace(MOD_ENG,TRACE_INFO, "SID130B_PCLK_Pre:%d",clk);
	}
	if (SID130B_Video_Mode == TRUE)
	{		
		max_fps =  SID130B_Min_Fps_Video;
	}
	else
	{		
		max_fps = SID130B_MAX_CAMERA_FPS;
	}
	
    Dummy_Line = SID130B_PCLK_Pre * SID130B_FRAME_RATE_UNIT / (2 * Line_length * max_fps) - SID130B_PV_PERIOD_LINE_NUMS; 
    if(Dummy_Line > SID130B_BLANK_REGISTER_LIMITATION)
    {
        Dummy_Line = SID130B_BLANK_REGISTER_LIMITATION;
        Line_length = SID130B_PCLK_Pre * SID130B_FRAME_RATE_UNIT / (2 * (Dummy_Line + SID130B_PV_PERIOD_LINE_NUMS) * max_fps);
    }	
	else if(Dummy_Line < 0)
	{
		max_fps = SID130B_PCLK_Pre * SID130B_FRAME_RATE_UNIT / (2 * Line_length * SID130B_PV_PERIOD_LINE_NUMS);
	}
    SID130B_PV_Hblank = Line_length -  SID130B_PV_PERIOD_PIXEL_NUMS;
    SID130B_Config_PV_Blank((SID130B_PV_Hblank > 0) ? SID130B_PV_Hblank : 0, (Dummy_Line > 0) ? Dummy_Line : 0);
    
}




void SID130B_SID130B_HVMirror(kal_uint8 image_mirror)
{
    kal_uint8 iHV_Mirror;
    
    if(SID130B_HVMirror == image_mirror)
        return;
    
    SID130B_HVMirror = image_mirror;
	//SID130B_set_page(0x00);
    iHV_Mirror = (SID130B_Control & 0xFC);  
    switch (image_mirror) 
    {
        case IMAGE_NORMAL:
            iHV_Mirror |= 0x00;
            break;
        case IMAGE_H_MIRROR:
            iHV_Mirror |= 0x01;
            break;
        case IMAGE_V_MIRROR:
            iHV_Mirror |= 0x02;
            break;
        case IMAGE_HV_MIRROR:
            iHV_Mirror |= 0x03;
            break;
        default:
            iHV_Mirror |= 0x00;
    }
    //SID130B_write_cmos_sensor(0x04, iHV_Mirror);
    SID130B_Control = iHV_Mirror;
}   /* SID130B_SID130B_HVMirror */



static void SID130B_FixFrameRate(kal_bool bEnable)
{
    if(bEnable == TRUE)
    {   //fix frame rate
        SID130B_Control |= 0xC0;
        //SID130B_set_page(0);
        //SID130B_write_cmos_sensor(0x04,SID130B_Control);
    }
    else
    {        
        SID130B_Control &= 0x3F;
        //SID130B_set_page(0);
        //SID130B_write_cmos_sensor(0x04,SID130B_Control);

    }
}   /* SID130B_FixFrameRate */

void SID130B_set_dummy(kal_uint16 pixels, kal_uint16 lines)
{
    kal_uint8 temp_reg1, temp_reg2;
    kal_uint16 temp_reg;

    /*Very Important: The line_length must < 0x1000, it is to say 0x3028 must < 0x10, or else the sensor will crash*/
    /*The dummy_pixel must < 2156*/
    if (pixels >= 2156) 
        pixels = 2155;
    if (pixels < 0x100)
    {
        SID130B_write_cmos_sensor(0x302c,(pixels&0xFF)); //EXHTS[7:0]
        temp_reg = SID130B_FULL_PERIOD_PIXEL_NUMS;
        SID130B_write_cmos_sensor(0x3029,(temp_reg&0xFF));         //H_length[7:0]
        SID130B_write_cmos_sensor(0x3028,((temp_reg&0xFF00)>>8));  //H_length[15:8]
    }
    else
    {
        SID130B_write_cmos_sensor(0x302c,0);
        temp_reg = pixels + SID130B_FULL_PERIOD_PIXEL_NUMS;
        SID130B_write_cmos_sensor(0x3029,(temp_reg&0xFF));         //H_length[7:0]
        SID130B_write_cmos_sensor(0x3028,((temp_reg&0xFF00)>>8));  //H_length[15:8]
    }

    // read out and + line
    temp_reg1 = SID130B_read_cmos_sensor(0x302B);    // VTS[b7~b0]
    temp_reg2 = SID130B_read_cmos_sensor(0x302A);    // VTS[b15~b8]
    temp_reg = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

    temp_reg += lines;

    SID130B_write_cmos_sensor(0x302B,(temp_reg&0xFF));         //VTS[7:0]
    SID130B_write_cmos_sensor(0x302A,((temp_reg&0xFF00)>>8));  //VTS[15:8]
}    /* SID130B_set_dummy */

void SID130B_NightMode(kal_bool bEnable)
{    
	SENSORDB("zhijie SID130B_NightMode ");

	if((SID130B_g_bNightMode == bEnable) && (SID130B_Mode_change == FALSE))
		return;
	
    SID130B_g_bNightMode = bEnable;
    SID130B_Mode_change = FALSE;        

    if (SID130B_g_bNightMode == TRUE)
    {   /* camera night mode */
        //SID130B_set_page(1);
		/* Max Analog Gain Value @ Shutter step = Max Shutter step */
		/*0x58 : 7X; 0x60 : 8X; 0x68 : 10X; 0x70 : 12X; 0x80 : 16X; 0xff : 32X;*/
        ///SID130B_write_cmos_sensor(0x40,0x70); 
        SID130B_set_page(4);
        SID130B_write_cmos_sensor(0xb6,0x0c); //Brightness Control 0x11
        //SID130B_write_cmos_sensor(0xb2,0x70); //Color Suppression Change Start State  0x17            
        //SID130B_write_cmos_sensor(0xb3,0x10); //Slope
    }
    else
    {   /* camera normal mode */
        //SID130B_set_page(1);
        //SID130B_write_cmos_sensor(0x40,0x70); //Max Analog Gain Value @ Shutter step = Max Shutter step  0x7D
		SID130B_set_page(4);
        SID130B_write_cmos_sensor(0xb6,0x04); //Brightness Control 0x11
        //SID130B_write_cmos_sensor(0xb2,0x30); //Color Suppression Change Start State  0x17            
        //SID130B_write_cmos_sensor(0xb3,0x10); //Slope
    }
        
    SID130B_Set_Frame_Count();     
}   /* SID130B_NightMode */

static void SID130B_YUV_sensor_initial_setting(void)
{
#if 0
	SID130B_set_page(0x00); 
	SID130B_write_cmos_sensor(0x04, 0x00); //Select Group B
	SID130B_write_cmos_sensor(0x05, 0x0F); //Select Group A :UXGA Output
	SID130B_write_cmos_sensor(0x06, 0x8E); //Select Group B :SVGA Output
	
	// Vendor recommended value ### Don't change ###
#ifdef SID130B_MT6253_EVB
	SID130B_write_cmos_sensor(0x09, 0x04); 
	SID130B_write_cmos_sensor(0x0A, 0x04); 
	SID130B_write_cmos_sensor(0x08, 0xa2); //PLL on
#else
#ifdef SID130B_DEBUG
	SID130B_write_cmos_sensor(0x09, pllset); 
	SID130B_write_cmos_sensor(0x0A, 0x03); 
	SID130B_write_cmos_sensor(0x08, pll); //PLL on
#else
	SID130B_write_cmos_sensor(0x09, 0x03); //.
	SID130B_write_cmos_sensor(0x0A, 0x03); //.0x03
	SID130B_write_cmos_sensor(0x08, 0xa2); //PLL on
#endif	
#endif	
	SID130B_write_cmos_sensor(0x10, 0x27);//17
	SID130B_write_cmos_sensor(0x11, 0x03);//02
	SID130B_write_cmos_sensor(0x12, 0x89); //0x8a 2010.02.08//86

	SID130B_write_cmos_sensor(0x13, 0x1a);
	SID130B_write_cmos_sensor(0x14, 0x07);//27
	SID130B_write_cmos_sensor(0x15, 0x22);//22
	SID130B_write_cmos_sensor(0x16, 0x40);//40
	
	SID130B_write_cmos_sensor(0x17, 0xCF); //cb
	SID130B_write_cmos_sensor(0x18, 0x38); 
	SID130B_write_cmos_sensor(0x40, 0x0F);
	SID130B_write_cmos_sensor(0x41, 0x17);
	SID130B_write_cmos_sensor(0x42, 0x52);
	
	//Flicker - 50Hz  - Still mode	 48MHz MCLK    
	SID130B_write_cmos_sensor(0x20, 0x00);
	SID130B_write_cmos_sensor(0x21, 0x02);
	SID130B_write_cmos_sensor(0x23, 0x51);
	
	//Flicker - 50Hz - Preview mode 48MHz MCLK
	SID130B_write_cmos_sensor(0x24, 0x00);
	SID130B_write_cmos_sensor(0x25, 0x10);
	SID130B_write_cmos_sensor(0x27, 0x2A);


	
	SID130B_set_page(0x01); 
	SID130B_write_cmos_sensor(0x34, 0xC8); 
	SID130B_write_cmos_sensor(0x35, 0x78);
	SID130B_write_cmos_sensor(0x36, 0x26);
	
	//AE Block 
	SID130B_write_cmos_sensor(0x10, 0x00);
	SID130B_write_cmos_sensor(0x11, 0x0A);
	SID130B_write_cmos_sensor(0x12, 0x78);
	SID130B_write_cmos_sensor(0x13, 0x78);
	SID130B_write_cmos_sensor(0x14, 0x78);
	SID130B_write_cmos_sensor(0x17, 0xC4);
	
	SID130B_write_cmos_sensor(0x19, 0x00);//0x8E
	SID130B_write_cmos_sensor(0x1A, 0x00);//0x0A
	
	
	SID130B_write_cmos_sensor(0x40, 0x70); //Max Again	//anyuan 0x73
	SID130B_write_cmos_sensor(0x41, 0x20);
	SID130B_write_cmos_sensor(0x42, 0x20);
	SID130B_write_cmos_sensor(0x43, 0x00);
	SID130B_write_cmos_sensor(0x44, 0x00);
	SID130B_write_cmos_sensor(0x45, 0x01);
	SID130B_write_cmos_sensor(0x46, 0x0C);
	SID130B_write_cmos_sensor(0x47, 0x11);
	SID130B_write_cmos_sensor(0x48, 0x15);
	SID130B_write_cmos_sensor(0x49, 0x17);
	SID130B_write_cmos_sensor(0x4A, 0x1A);
	SID130B_write_cmos_sensor(0x4B, 0x1C);
	SID130B_write_cmos_sensor(0x4C, 0x1E);
	SID130B_write_cmos_sensor(0x4D, 0x1E);
	SID130B_write_cmos_sensor(0x4E, 0x0F);
	SID130B_write_cmos_sensor(0x4F, 0x09);
	SID130B_write_cmos_sensor(0x50, 0x07);
	SID130B_write_cmos_sensor(0x51, 0x05);
	SID130B_write_cmos_sensor(0x52, 0x04);
	SID130B_write_cmos_sensor(0x53, 0x03);
	SID130B_write_cmos_sensor(0x54, 0x02);
	SID130B_write_cmos_sensor(0x55, 0x01);

	// NORMAL mode AE metering : window setting change
	SID130B_write_cmos_sensor(0x60, 0xFF);
	SID130B_write_cmos_sensor(0x61, 0xFF);		 
	SID130B_write_cmos_sensor(0x62, 0xFF);
	SID130B_write_cmos_sensor(0x63, 0xFF);
	SID130B_write_cmos_sensor(0x64, 0xFF);
	SID130B_write_cmos_sensor(0x65, 0xFF);
	
	SID130B_write_cmos_sensor(0x66, 0x00);
	SID130B_write_cmos_sensor(0x67, 0x50);		 
	SID130B_write_cmos_sensor(0x68, 0x50);
	SID130B_write_cmos_sensor(0x69, 0x50);
	SID130B_write_cmos_sensor(0x6A, 0x50);
	SID130B_write_cmos_sensor(0x6B, 0x00);
	SID130B_write_cmos_sensor(0x6C, 0x06);	
	
	SID130B_write_cmos_sensor(0x70, 0xc4);
	SID130B_write_cmos_sensor(0x73, 0x22);
	SID130B_write_cmos_sensor(0x74, 0x07);
	SID130B_write_cmos_sensor(0x77, 0xd0);
	SID130B_write_cmos_sensor(0x78, 0xd8);	
		  
	
	//AWB Block
	SID130B_set_page(0x02); 
	SID130B_write_cmos_sensor(0x10, 0xD3);
	SID130B_write_cmos_sensor(0x11, 0x11);
	SID130B_write_cmos_sensor(0x13, 0x7F);	//# AWB taget Cr
	SID130B_write_cmos_sensor(0x14, 0x80);  //AWB taget Cb 		
	SID130B_write_cmos_sensor(0x15, 0xEE);
	SID130B_write_cmos_sensor(0x16, 0x80);
	SID130B_write_cmos_sensor(0x17, 0xD0);
	SID130B_write_cmos_sensor(0x18, 0x80);
	SID130B_write_cmos_sensor(0x19, 0x98);
	SID130B_write_cmos_sensor(0x1A, 0x68);
	SID130B_write_cmos_sensor(0x1B, 0x98);
	SID130B_write_cmos_sensor(0x1C, 0x68);
	SID130B_write_cmos_sensor(0x1D, 0x8C);
	SID130B_write_cmos_sensor(0x1E, 0x74);
	SID130B_write_cmos_sensor(0x20, 0xF0);
	SID130B_write_cmos_sensor(0x21, 0x84);
	SID130B_write_cmos_sensor(0x22, 0xB4);
	SID130B_write_cmos_sensor(0x23, 0x20);
	SID130B_write_cmos_sensor(0x25, 0x20);
	SID130B_write_cmos_sensor(0x26, 0x05);
	SID130B_write_cmos_sensor(0x27, 0x63); // for outdoor fix gain enter
	SID130B_write_cmos_sensor(0x28, 0x63); // for outdoor fix gain quit
	SID130B_write_cmos_sensor(0x29, 0xAB); 
	SID130B_write_cmos_sensor(0x2A, 0x9D);
	SID130B_write_cmos_sensor(0x30, 0x00);
	SID130B_write_cmos_sensor(0x31, 0x10); 
	SID130B_write_cmos_sensor(0x32, 0x00); 
	SID130B_write_cmos_sensor(0x33, 0x10); 
	SID130B_write_cmos_sensor(0x34, 0x06); 
	SID130B_write_cmos_sensor(0x35, 0x30); 
	SID130B_write_cmos_sensor(0x36, 0x04); 
	SID130B_write_cmos_sensor(0x37, 0xa0); 
	SID130B_write_cmos_sensor(0x40, 0x01); 
	SID130B_write_cmos_sensor(0x41, 0x04); 
	SID130B_write_cmos_sensor(0x42, 0x08); 
	SID130B_write_cmos_sensor(0x43, 0x10); 
	SID130B_write_cmos_sensor(0x44, 0x13); 
	SID130B_write_cmos_sensor(0x45, 0x6b); 
	SID130B_write_cmos_sensor(0x46, 0x82); 
	
	//CMA change  -D65~A
	SID130B_write_cmos_sensor(0x53, 0x8a); //AWB R Gain for D30 to D20
	SID130B_write_cmos_sensor(0x54, 0xb6); //AWB B Gain for D30 to D20
	SID130B_write_cmos_sensor(0x55, 0x8d); //AWB R Gain for D20 to D30
	SID130B_write_cmos_sensor(0x56, 0xb5); //AWB B Gain for D20 to D30
	SID130B_write_cmos_sensor(0x57, 0xa0); //AWB R Gain for D65 to D30
	SID130B_write_cmos_sensor(0x58, 0x88); //AWB B Gain for D65 to D30
	SID130B_write_cmos_sensor(0x59, 0xa0); //AWB R Gain for D30 to D65
	SID130B_write_cmos_sensor(0x5A, 0xa8); //AWB B Gain for D30 to D65
	
	SID130B_write_cmos_sensor(0x64, 0x00); //T1~T2 RGAIN
	SID130B_write_cmos_sensor(0x65, 0x00); 
	SID130B_write_cmos_sensor(0x66, 0x00); //T3~T4 RGAIN
	SID130B_write_cmos_sensor(0x67, 0x00); 
	SID130B_write_cmos_sensor(0x68, 0xA5); //T5~T6 RGAIN
	SID130B_write_cmos_sensor(0x69, 0xB4); 
	SID130B_write_cmos_sensor(0x6A, 0xB3); //T7~T8 RGAIN
	SID130B_write_cmos_sensor(0x6B, 0xAC); 
	SID130B_write_cmos_sensor(0x6C, 0xB7); //T9~T10 RGAIN
	SID130B_write_cmos_sensor(0x6D, 0x98); 
	SID130B_write_cmos_sensor(0x6E, 0xBA); //T11~T12 RGAIN
	SID130B_write_cmos_sensor(0x6F, 0x90); 
	SID130B_write_cmos_sensor(0x70, 0xBF); //T13~T14 RGAIN
	SID130B_write_cmos_sensor(0x71, 0x9B); 
	SID130B_write_cmos_sensor(0x72, 0xCE); //T15~T16 RGAIN
	SID130B_write_cmos_sensor(0x73, 0x8C); 
	
	SID130B_write_cmos_sensor(0x74, 0x7F); //T1 White RGAIN BOT
	SID130B_write_cmos_sensor(0x75, 0x8C); //T1 White RGAIN TOP
	SID130B_write_cmos_sensor(0x76, 0xAD); //T1 White BGAIN BOT
	SID130B_write_cmos_sensor(0x77, 0xBA); //T1 White BGAIN TOP
	SID130B_write_cmos_sensor(0x78, 0x8F); //T2 White RGAIN BOT
	SID130B_write_cmos_sensor(0x79, 0x9A); //T2 White RGAIN TOP
	SID130B_write_cmos_sensor(0x7A, 0xA3); //T2 White BGAIN BOT
	SID130B_write_cmos_sensor(0x7B, 0xAC); //T2 White BGAIN TOP
	SID130B_write_cmos_sensor(0x7C, 0xA0); //T3 White RGAIN BOT
	SID130B_write_cmos_sensor(0x7D, 0xA9); //T3 White RGAIN TOP
	SID130B_write_cmos_sensor(0x7E, 0x95); //T3 White BGAIN BOT
	SID130B_write_cmos_sensor(0x7F, 0xAC); //T3 White BGAIN TOP
	SID130B_write_cmos_sensor(0x80, 0xAD); //T4 White RGAIN BOT
	SID130B_write_cmos_sensor(0x81, 0xBC); //T4 White RGAIN TOP
	SID130B_write_cmos_sensor(0x82, 0x98); //T4 White BGAIN BOT
	SID130B_write_cmos_sensor(0x83, 0xA4); //T4 White BGAIN TOP
	SID130B_write_cmos_sensor(0x84, 0x00); //T5 White RGAIN BOT
	SID130B_write_cmos_sensor(0x85, 0x00); //T5 White RGAIN TOP
	SID130B_write_cmos_sensor(0x86, 0x00); //T5 White BGAIN BOT
	SID130B_write_cmos_sensor(0x87, 0x00); //T5 White BGAIN TOP
	SID130B_write_cmos_sensor(0x88, 0xC9); //T6 White RGAIN BOT
	SID130B_write_cmos_sensor(0x89, 0xD5); //T6 White RGAIN TOP
	SID130B_write_cmos_sensor(0x8A, 0x70); //T6 White BGAIN BOT
	SID130B_write_cmos_sensor(0x8B, 0x7B); //T6 White BGAIN TOP
	SID130B_write_cmos_sensor(0x8C, 0xD0); //T7 White RGAIN BOT
	SID130B_write_cmos_sensor(0x8D, 0xE5); //T7 White RGAIN TOP
	SID130B_write_cmos_sensor(0x8E, 0x58); //T7 White BGAIN BOT
	SID130B_write_cmos_sensor(0x8F, 0x70); //T7 White BGAIN TOP
	
	SID130B_write_cmos_sensor(0xB4, 0x05); 
	SID130B_write_cmos_sensor(0xB5, 0x0F); 
	SID130B_write_cmos_sensor(0xB6, 0x06); 
	SID130B_write_cmos_sensor(0xB7, 0x06); 
	SID130B_write_cmos_sensor(0xB8, 0x40); 
	SID130B_write_cmos_sensor(0xB9, 0x10); 
	SID130B_write_cmos_sensor(0xBA, 0x06); 
	
	//IDP
	SID130B_set_page(0x03); 
	SID130B_write_cmos_sensor(0x10, 0xFF);
	SID130B_write_cmos_sensor(0x11, 0x1D);//1d
	SID130B_write_cmos_sensor(0x12, 0x1D);
	SID130B_write_cmos_sensor(0x13, 0xFF);
	SID130B_write_cmos_sensor(0x14, 0x00);
	SID130B_write_cmos_sensor(0x15, 0xc0);
	//DPC
	SID130B_write_cmos_sensor(0x30, 0x88); //DPCNRCTRL
	SID130B_write_cmos_sensor(0x31, 0x10); //DPTHR @ AGAIN = 00
	SID130B_write_cmos_sensor(0x32, 0x0c); //DPTHR @ AGAIN = 20
	SID130B_write_cmos_sensor(0x33, 0x08); //DPTHR @ AGAIN = 40
	SID130B_write_cmos_sensor(0x34, 0x04); //DPTHR @ AGAIN = 60
	SID130B_write_cmos_sensor(0x35, 0x00); //DPTHR @ AGAIN = 80
	SID130B_write_cmos_sensor(0x36, 0x44); //DPTHVRNG
	SID130B_write_cmos_sensor(0x37, 0x66); //DPNUMBER
	SID130B_write_cmos_sensor(0x38, 0x00); //0x00	# NRTHR0 @ AGAIN = 00
	SID130B_write_cmos_sensor(0x39, 0x18); //0x0C	# NRTHR1 @ AGAIN = 20
	SID130B_write_cmos_sensor(0x3A, 0x40); //0x18	# NRTHR2 @ AGAIN = 40
	SID130B_write_cmos_sensor(0x3B, 0x80); //0x30	# NRTHR3 @ AGAIN = 60
	SID130B_write_cmos_sensor(0x3C, 0x80); //		   NRTHR4 @ AGAIN = 80
	SID130B_write_cmos_sensor(0x3D, 0x04); //NRTHVRNG0 @ AGAIN = 00
	SID130B_write_cmos_sensor(0x3E, 0x0c); //NRTHVRNG1 @ AGAIN = 20
	SID130B_write_cmos_sensor(0x3F, 0x20); //NRTHVRNG2 @ AGAIN = 40
	SID130B_write_cmos_sensor(0x40, 0x80); //NRTHVRNG3 @ AGAIN = 60
	SID130B_write_cmos_sensor(0x41, 0x80); //NRTHVRNG4 @ AGAIN = 80
	SID130B_write_cmos_sensor(0x42, 0x10); //NRTHVRNGMAX
	SID130B_write_cmos_sensor(0x43, 0x40); //NRTHRWGT
	SID130B_write_cmos_sensor(0x44, 0x40); //BASELVL
	SID130B_write_cmos_sensor(0x45, 0x06); //SHUMAXH
	SID130B_write_cmos_sensor(0x46, 0x40); //SHUMAXL
	SID130B_write_cmos_sensor(0x47, 0x30); //ILLUMITHDRK
	
	//Shading
	SID130B_write_cmos_sensor(0x50, 0x20); //RXSHDL
	SID130B_write_cmos_sensor(0x51, 0x32); //RXSHDR
	SID130B_write_cmos_sensor(0x52, 0x40); //RXSHDT
	SID130B_write_cmos_sensor(0x53, 0x2a); //RXSHDB 	
	SID130B_write_cmos_sensor(0x54, 0x1b); //GXSHDL
	SID130B_write_cmos_sensor(0x55, 0x19); //GXSHDR
	SID130B_write_cmos_sensor(0x56, 0x20); //GXSHDT
	SID130B_write_cmos_sensor(0x57, 0x15); //GXSHDB	
	SID130B_write_cmos_sensor(0x58, 0x1b); //GXSHDL
	SID130B_write_cmos_sensor(0x59, 0x19); //GXSHDR
	SID130B_write_cmos_sensor(0x5A, 0x20); //GXSHDT
	SID130B_write_cmos_sensor(0x5B, 0x15); //GXSHDB	
	SID130B_write_cmos_sensor(0x5C, 0x33); //BXSHDL
	SID130B_write_cmos_sensor(0x5D, 0x42); //BXSHDR
	SID130B_write_cmos_sensor(0x5E, 0x49); //BXSHDT
	SID130B_write_cmos_sensor(0x5F, 0x30); //BXSHDB	
	SID130B_write_cmos_sensor(0x60, 0x32); //#SHDCNTH(X/Y)
	SID130B_write_cmos_sensor(0x61, 0x20); //#SHDCNTX
	SID130B_write_cmos_sensor(0x62, 0x67); //#SHDCNTY	
	SID130B_write_cmos_sensor(0x63, 0x8A); 
	SID130B_write_cmos_sensor(0x66, 0x76);	
	SID130B_write_cmos_sensor(0x6B, 0x01);
	SID130B_write_cmos_sensor(0x6C, 0x22);
	SID130B_write_cmos_sensor(0x6D, 0x23);
	SID130B_write_cmos_sensor(0x6E, 0x55);
	SID130B_write_cmos_sensor(0x6F, 0x77);
	SID130B_write_cmos_sensor(0x70, 0x65);
	SID130B_write_cmos_sensor(0x71, 0x01);
	SID130B_write_cmos_sensor(0x72, 0x23);
	SID130B_write_cmos_sensor(0x73, 0x33);
	SID130B_write_cmos_sensor(0x74, 0x45);
	SID130B_write_cmos_sensor(0x75, 0x55);
	SID130B_write_cmos_sensor(0x76, 0x55);
	SID130B_write_cmos_sensor(0x77, 0x01);
	SID130B_write_cmos_sensor(0x78, 0x23);
	SID130B_write_cmos_sensor(0x79, 0x33);
	SID130B_write_cmos_sensor(0x7A, 0x45);
	SID130B_write_cmos_sensor(0x7B, 0x55);
	SID130B_write_cmos_sensor(0x7C, 0x55);
	SID130B_write_cmos_sensor(0x7D, 0x00);
	SID130B_write_cmos_sensor(0x7E, 0x01);
	SID130B_write_cmos_sensor(0x7F, 0x11);
	SID130B_write_cmos_sensor(0x80, 0x33);
	SID130B_write_cmos_sensor(0x81, 0x33);
	SID130B_write_cmos_sensor(0x82, 0x22);
	SID130B_write_cmos_sensor(0x83, 0x14);
	SID130B_write_cmos_sensor(0x84, 0x0f);
	
	//SID130B_write_cmos_sensor(0x94, 0x06);
	//SID130B_write_cmos_sensor(0x95, 0x40);
	//SID130B_write_cmos_sensor(0x96, 0x04);
	//SID130B_write_cmos_sensor(0x97, 0xb0);
	
	
	//Interpolation
	SID130B_write_cmos_sensor(0xA0, 0x2F);
	SID130B_write_cmos_sensor(0xA1, 0x04);
	SID130B_write_cmos_sensor(0xA2, 0xB7);
	SID130B_write_cmos_sensor(0xA3, 0xB7);
	SID130B_write_cmos_sensor(0xA4, 0x08);
	SID130B_write_cmos_sensor(0xA5, 0xFF);
	SID130B_write_cmos_sensor(0xA6, 0x02);
	SID130B_write_cmos_sensor(0xA7, 0xFF);
	SID130B_write_cmos_sensor(0xA8, 0x00);
	SID130B_write_cmos_sensor(0xA9, 0x00);
	SID130B_write_cmos_sensor(0xAA, 0x00);
	SID130B_write_cmos_sensor(0xAB, 0x00);
	SID130B_write_cmos_sensor(0xAC, 0x60);
	SID130B_write_cmos_sensor(0xAD, 0x18);
	SID130B_write_cmos_sensor(0xAE, 0x10);
	SID130B_write_cmos_sensor(0xAF, 0x20);
	SID130B_write_cmos_sensor(0xB0, 0x08);
	SID130B_write_cmos_sensor(0xB1, 0x00);
	
	//Color Matrix for D65
	SID130B_write_cmos_sensor(0xC0, 0xaF);// CMASB D20 or D30 or Dark Condition Color Matrix Selection
	SID130B_write_cmos_sensor(0xC1, 0x66);
	SID130B_write_cmos_sensor(0xC2, 0xd4);
	SID130B_write_cmos_sensor(0xC3, 0x06);
	SID130B_write_cmos_sensor(0xC4, 0xF0);
	SID130B_write_cmos_sensor(0xC5, 0x5A);
	SID130B_write_cmos_sensor(0xC6, 0xFA);
	SID130B_write_cmos_sensor(0xC7, 0xF9);
	SID130B_write_cmos_sensor(0xC8, 0xBF);
	SID130B_write_cmos_sensor(0xC9, 0x88);
	SID130B_write_cmos_sensor(0xCA, 0x00);
	SID130B_write_cmos_sensor(0xCB, 0x00);
	SID130B_write_cmos_sensor(0xCC, 0x00);
	SID130B_write_cmos_sensor(0xCD, 0x00);
	SID130B_write_cmos_sensor(0xCE, 0x00);
	
	//Color Matrix for CWF
	SID130B_write_cmos_sensor(0xD0, 0x2F);
	SID130B_write_cmos_sensor(0xD1, 0x71);
	SID130B_write_cmos_sensor(0xD2, 0xb6);
	SID130B_write_cmos_sensor(0xD3, 0x19);
	SID130B_write_cmos_sensor(0xD4, 0xE8);
	SID130B_write_cmos_sensor(0xD5, 0x5a);
	SID130B_write_cmos_sensor(0xD6, 0xFe);
	SID130B_write_cmos_sensor(0xD7, 0xe8);
	SID130B_write_cmos_sensor(0xD8, 0xae);
	SID130B_write_cmos_sensor(0xD9, 0xaa);
	SID130B_write_cmos_sensor(0xDA, 0x00);
	SID130B_write_cmos_sensor(0xDB, 0x00);
	SID130B_write_cmos_sensor(0xDC, 0x00);
	SID130B_write_cmos_sensor(0xDD, 0x00);
	SID130B_write_cmos_sensor(0xDE, 0x00);
	
	//Color Matrix for A
	SID130B_write_cmos_sensor(0xE0, 0x2F);
	SID130B_write_cmos_sensor(0xE1, 0x5C);
	SID130B_write_cmos_sensor(0xE2, 0xDD);
	SID130B_write_cmos_sensor(0xE3, 0x06);
	SID130B_write_cmos_sensor(0xE4, 0xE0);
	SID130B_write_cmos_sensor(0xE5, 0x69);
	SID130B_write_cmos_sensor(0xE6, 0xF6);
	SID130B_write_cmos_sensor(0xE7, 0xE5);
	SID130B_write_cmos_sensor(0xE8, 0xAB);
	SID130B_write_cmos_sensor(0xE9, 0xAE);
	SID130B_write_cmos_sensor(0xEA, 0x6A);
	SID130B_write_cmos_sensor(0xEB, 0x01);
	SID130B_write_cmos_sensor(0xEC, 0x2D);
	SID130B_write_cmos_sensor(0xED, 0xEE);
	SID130B_write_cmos_sensor(0xEE, 0x04);
	
	//IDP 2
	SID130B_set_page(0x04); 	
	//Gamma - R
	SID130B_write_cmos_sensor(0x10, 0x00);
	SID130B_write_cmos_sensor(0x11, 0x06);
	SID130B_write_cmos_sensor(0x12, 0x10);
	SID130B_write_cmos_sensor(0x13, 0x24);
	SID130B_write_cmos_sensor(0x14, 0x48);
	SID130B_write_cmos_sensor(0x15, 0x64);
	SID130B_write_cmos_sensor(0x16, 0x7c);
	SID130B_write_cmos_sensor(0x17, 0x8e);
	SID130B_write_cmos_sensor(0x18, 0xa0);
	SID130B_write_cmos_sensor(0x19, 0xae);
	SID130B_write_cmos_sensor(0x1A, 0xbb);
	SID130B_write_cmos_sensor(0x1B, 0xd0);
	SID130B_write_cmos_sensor(0x1C, 0xe2);
	SID130B_write_cmos_sensor(0x1D, 0xf2);
	SID130B_write_cmos_sensor(0x1E, 0xf9);
	SID130B_write_cmos_sensor(0x1F, 0xFF);
							
	//Gamma - G 			
	SID130B_write_cmos_sensor(0x20, 0x00);
	SID130B_write_cmos_sensor(0x21, 0x06);
	SID130B_write_cmos_sensor(0x22, 0x10);
	SID130B_write_cmos_sensor(0x23, 0x24);
	SID130B_write_cmos_sensor(0x24, 0x48);
	SID130B_write_cmos_sensor(0x25, 0x64);
	SID130B_write_cmos_sensor(0x26, 0x7c);
	SID130B_write_cmos_sensor(0x27, 0x8e);
	SID130B_write_cmos_sensor(0x28, 0xa0);
	SID130B_write_cmos_sensor(0x29, 0xae);
	SID130B_write_cmos_sensor(0x2A, 0xbb);
	SID130B_write_cmos_sensor(0x2B, 0xd0);
	SID130B_write_cmos_sensor(0x2C, 0xe2);
	SID130B_write_cmos_sensor(0x2D, 0xf2);
	SID130B_write_cmos_sensor(0x2E, 0xf9);
	SID130B_write_cmos_sensor(0x2F, 0xFF);
							
	//Gamma - B 			
	SID130B_write_cmos_sensor(0x30, 0x00);
	SID130B_write_cmos_sensor(0x31, 0x06);
	SID130B_write_cmos_sensor(0x32, 0x10);
	SID130B_write_cmos_sensor(0x33, 0x24);
	SID130B_write_cmos_sensor(0x34, 0x48);
	SID130B_write_cmos_sensor(0x35, 0x64);
	SID130B_write_cmos_sensor(0x36, 0x7c);
	SID130B_write_cmos_sensor(0x37, 0x8e);
	SID130B_write_cmos_sensor(0x38, 0xa0);
	SID130B_write_cmos_sensor(0x39, 0xae);
	SID130B_write_cmos_sensor(0x3A, 0xbb);
	SID130B_write_cmos_sensor(0x3B, 0xd0);
	SID130B_write_cmos_sensor(0x3C, 0xe2);
	SID130B_write_cmos_sensor(0x3D, 0xf2);
	SID130B_write_cmos_sensor(0x3E, 0xf9);
	SID130B_write_cmos_sensor(0x3F, 0xFF);
							
	//CSC					
	SID130B_write_cmos_sensor(0x60, 0x33);
	SID130B_write_cmos_sensor(0x61, 0x20);
	SID130B_write_cmos_sensor(0x62, 0xE4);
	SID130B_write_cmos_sensor(0x63, 0xFA);
	SID130B_write_cmos_sensor(0x64, 0x13);
	SID130B_write_cmos_sensor(0x65, 0x25);
	SID130B_write_cmos_sensor(0x66, 0x07);
	SID130B_write_cmos_sensor(0x67, 0xF5);
	SID130B_write_cmos_sensor(0x68, 0xEA);
	SID130B_write_cmos_sensor(0x69, 0x20);
	SID130B_write_cmos_sensor(0x6A, 0xC8);
	SID130B_write_cmos_sensor(0x6B, 0xC4);
	SID130B_write_cmos_sensor(0x6C, 0x84);
	SID130B_write_cmos_sensor(0x6D, 0x04);
	SID130B_write_cmos_sensor(0x6E, 0x0C);
	SID130B_write_cmos_sensor(0x6F, 0x00);
							
	//Edge					
	SID130B_write_cmos_sensor(0x80, 0xA2);
	SID130B_write_cmos_sensor(0x81, 0x10);
	SID130B_write_cmos_sensor(0x82, 0x14);
	SID130B_write_cmos_sensor(0x83, 0x04);
	SID130B_write_cmos_sensor(0x84, 0x18);
	SID130B_write_cmos_sensor(0x85, 0x06);
	SID130B_write_cmos_sensor(0x86, 0x00);
	SID130B_write_cmos_sensor(0x87, 0x04);
	SID130B_write_cmos_sensor(0x88, 0x18);
	SID130B_write_cmos_sensor(0x89, 0x06);
	SID130B_write_cmos_sensor(0x8a, 0x00);
	SID130B_write_cmos_sensor(0x8b, 0x24);
	SID130B_write_cmos_sensor(0x8c, 0x24);
	SID130B_write_cmos_sensor(0X90, 0x16);
	SID130B_write_cmos_sensor(0x91, 0x03);
	SID130B_write_cmos_sensor(0x93, 0xE0);
	
	//Cr/Cb Coring
	SID130B_write_cmos_sensor(0x94, 0x12);
	SID130B_write_cmos_sensor(0x95, 0x12);
	SID130B_write_cmos_sensor(0x96, 0x4C);
	SID130B_write_cmos_sensor(0x97, 0x76);
	SID130B_write_cmos_sensor(0x9A, 0xF5);
	SID130B_write_cmos_sensor(0xA1, 0x08);
	SID130B_write_cmos_sensor(0xA2, 0x10);
	SID130B_write_cmos_sensor(0xA3, 0x16);
	SID130B_write_cmos_sensor(0xA4, 0x20);
	SID130B_write_cmos_sensor(0xA5, 0x30);
	SID130B_write_cmos_sensor(0xA6, 0xA0);
	SID130B_write_cmos_sensor(0xA7, 0x09);
	SID130B_write_cmos_sensor(0xA8, 0x60);	
	SID130B_write_cmos_sensor(0xA9, 0x20);
	SID130B_write_cmos_sensor(0xAA, 0x60);	
	SID130B_write_cmos_sensor(0xAC, 0xFF);
	SID130B_write_cmos_sensor(0xAD, 0x09);
	SID130B_write_cmos_sensor(0xAE, 0x96);
	SID130B_write_cmos_sensor(0xAF, 0x18);
	
	SID130B_write_cmos_sensor(0xB2, 0x30); //color suppression start level	//0x40//50
	SID130B_write_cmos_sensor(0xB3, 0x14); //[7:4]color suppression slope //0x33//15
										   //[3:0]color suppression end gain
	//SID130B_write_cmos_sensor(0xB6, 0x08); //Brightness Control
	
	//Color Saturation
	SID130B_write_cmos_sensor(0xBC, 0x15); //0x14
	SID130B_write_cmos_sensor(0xBD, 0x15);
	SID130B_write_cmos_sensor(0xBE, 0x16);
	SID130B_write_cmos_sensor(0xBF, 0x16); 
	
    SID130B_write_cmos_sensor(0xc0, 0x10);
	SID130B_write_cmos_sensor(0xc1, 0x10);
	SID130B_write_cmos_sensor(0xc2, 0x14);
	SID130B_write_cmos_sensor(0xc3, 0x14);
	SID130B_write_cmos_sensor(0xc4, 0x14);
	SID130B_write_cmos_sensor(0xc5, 0x14);
	SID130B_write_cmos_sensor(0xc6, 0x01);
	SID130B_write_cmos_sensor(0xc7, 0x01);
	SID130B_write_cmos_sensor(0xc8, 0x01);
	SID130B_write_cmos_sensor(0xc9, 0x01);
	SID130B_write_cmos_sensor(0xca, 0x01);
	SID130B_write_cmos_sensor(0xcb, 0x01);
	SID130B_write_cmos_sensor(0xcc, 0x04);
	SID130B_write_cmos_sensor(0xcd, 0x3f);
	SID130B_write_cmos_sensor(0xce, 0x01);	
							
	//IDP 3 				
	SID130B_set_page(0x05); 
							
	//Memory				
	SID130B_write_cmos_sensor(0x40, 0x15);
	SID130B_write_cmos_sensor(0x41, 0x28);
	SID130B_write_cmos_sensor(0x42, 0x04);
	SID130B_write_cmos_sensor(0x43, 0x15);
	SID130B_write_cmos_sensor(0x44, 0x28);
	SID130B_write_cmos_sensor(0x45, 0x04);
	SID130B_write_cmos_sensor(0x46, 0x15);
	SID130B_write_cmos_sensor(0x47, 0x28);
	SID130B_write_cmos_sensor(0x48, 0x04);
							
	//Knee					
	SID130B_write_cmos_sensor(0x90, 0xCA); //
	SID130B_write_cmos_sensor(0x91, 0x81); //knee function selection/knee point H
	SID130B_write_cmos_sensor(0x92, 0x00); //knee point L
	SID130B_write_cmos_sensor(0x93, 0x50); //Knee gain
	SID130B_write_cmos_sensor(0x94, 0x41); //[6:4]knee start H/[2:0]Knee END H
	SID130B_write_cmos_sensor(0x95, 0x7E); //knee start L
	SID130B_write_cmos_sensor(0x96, 0x48); //knee END L
							
	//ADG					
	SID130B_write_cmos_sensor(0x99, 0xC0);
	SID130B_write_cmos_sensor(0xA0, 0x10);
	SID130B_write_cmos_sensor(0xA1, 0x22);
	SID130B_write_cmos_sensor(0xA2, 0x36);
	SID130B_write_cmos_sensor(0xA3, 0x49);
	SID130B_write_cmos_sensor(0xA4, 0x5D);
	SID130B_write_cmos_sensor(0xA5, 0x70);
	SID130B_write_cmos_sensor(0xA6, 0x82);
	SID130B_write_cmos_sensor(0xA7, 0x94);
	SID130B_write_cmos_sensor(0xA8, 0xA5);
	SID130B_write_cmos_sensor(0xA9, 0xB5);
	SID130B_write_cmos_sensor(0xAA, 0xC3);
	SID130B_write_cmos_sensor(0xAB, 0xD1);
	SID130B_write_cmos_sensor(0xAC, 0xDE);
	SID130B_write_cmos_sensor(0xAD, 0xEA);
	SID130B_write_cmos_sensor(0xAE, 0xF5);
	SID130B_write_cmos_sensor(0xAF, 0xFF);
							
	//YXGMA 				
	SID130B_write_cmos_sensor(0xB0, 0xc0); //YGMACTRL
	SID130B_write_cmos_sensor(0xB1, 0x04); //YGMASLOP
	SID130B_write_cmos_sensor(0xB8, 0x0f); //DRKTHR1
	SID130B_write_cmos_sensor(0xB9, 0x10); //DRKTHR2
	//SID130B_write_cmos_sensor(0xBA,	0x38); //DRKTHR3
	//SID130B_write_cmos_sensor(0xBB,	0x39); //DRKTHR4
	SID130B_write_cmos_sensor(0xC0, 0x03);
	SID130B_write_cmos_sensor(0xC1, 0x0E);
	SID130B_write_cmos_sensor(0xC2, 0x16);
	SID130B_write_cmos_sensor(0xC3, 0x24);
	SID130B_write_cmos_sensor(0xC4, 0x3F);
	SID130B_write_cmos_sensor(0xC5, 0x56);
	SID130B_write_cmos_sensor(0xC6, 0x6A);
	SID130B_write_cmos_sensor(0xC7, 0x7C);
	SID130B_write_cmos_sensor(0xC8, 0x8C);
	SID130B_write_cmos_sensor(0xC9, 0x98);
	SID130B_write_cmos_sensor(0xCA, 0xA2);
	SID130B_write_cmos_sensor(0xCB, 0xB8);
	SID130B_write_cmos_sensor(0xCC, 0xCD);
	SID130B_write_cmos_sensor(0xCD, 0xE2);
	SID130B_write_cmos_sensor(0xCE, 0xF0);
	SID130B_write_cmos_sensor(0xCF, 0xFF);
	
	SID130B_write_cmos_sensor(0xe0, 0x04);
	SID130B_write_cmos_sensor(0xe1, 0x88);
	SID130B_write_cmos_sensor(0xe2, 0x09);
	SID130B_write_cmos_sensor(0xe3, 0x0c);
	SID130B_write_cmos_sensor(0xe4, 0x11);
	SID130B_write_cmos_sensor(0xe5, 0x16);
	SID130B_write_cmos_sensor(0xe6, 0x1b);
	SID130B_write_cmos_sensor(0xe7, 0x24);
	SID130B_write_cmos_sensor(0xe8, 0x30);
	SID130B_write_cmos_sensor(0xe9, 0x3e);	
	
	//Sensor on
	SID130B_set_page(0x00);
#ifdef SID130B_MT6253_EVB
	SID130B_write_cmos_sensor(0x03, 0x85);
#else
#ifdef SID130B_DEBUG
	SID130B_write_cmos_sensor(0x03, (sensor_current << 4) | 0x5);
#else
	SID130B_write_cmos_sensor(0x03, 0x95);//55
#endif	
#endif	
	SID130B_set_page(0x01); 
	SID130B_write_cmos_sensor(0x10, 0xc0);
#else
SID130B_set_page(0x00); 
SID130B_write_cmos_sensor(0x04, 0x00); //Select Group B
SID130B_write_cmos_sensor(0x05, 0x0F); //Select Group A :UXGA Output
SID130B_write_cmos_sensor(0x06, 0x8E); //Select Group B :SVGA Output                
// Vendor recommended value ### Don't change ###

SID130B_write_cmos_sensor(0x09, 0x11); 
SID130B_write_cmos_sensor(0x0A, 0x03); 
SID130B_write_cmos_sensor(0x08, 0xa2); //PLL on
 
SID130B_write_cmos_sensor(0x10, 0x27);//17
SID130B_write_cmos_sensor(0x11, 0x03);//02
SID130B_write_cmos_sensor(0x12, 0x89); //0x8a 2010.02.08//86
//SID130B_write_cmos_sensor(0x13, 0x1a);
SID130B_write_cmos_sensor(0x14, 0x07);//27
SID130B_write_cmos_sensor(0x15, 0x22);//22
SID130B_write_cmos_sensor(0x16, 0x40);//40
SID130B_write_cmos_sensor(0x17, 0xCF); //cb
SID130B_write_cmos_sensor(0x18, 0x38); 
SID130B_write_cmos_sensor(0x40, 0x0F);
SID130B_write_cmos_sensor(0x41, 0x17);
SID130B_write_cmos_sensor(0x42, 0x52);   
             
//Flicker - 50Hz  - Still mode       48MHz MCLK    
SID130B_write_cmos_sensor(0x20, 0x00);
SID130B_write_cmos_sensor(0x21, 0x02);
SID130B_write_cmos_sensor(0x23, 0x51);               

//Flicker - 50Hz - Preview mode 48MHz MCLK
SID130B_write_cmos_sensor(0x24, 0x00);
SID130B_write_cmos_sensor(0x25, 0x10);
SID130B_write_cmos_sensor(0x27, 0x2A); 

SID130B_set_page(0x01); 
SID130B_write_cmos_sensor(0x34, 0xC8); 
SID130B_write_cmos_sensor(0x35, 0x78);
SID130B_write_cmos_sensor(0x36, 0x26);
               
//AE Block 
SID130B_write_cmos_sensor(0x10, 0x00);
SID130B_write_cmos_sensor(0x11, 0x0C);//0A 100423
SID130B_write_cmos_sensor(0x12, 0x78);
SID130B_write_cmos_sensor(0x13, 0x78);
SID130B_write_cmos_sensor(0x14, 0x78);
SID130B_write_cmos_sensor(0x17, 0xC4);                
SID130B_write_cmos_sensor(0x19, 0x00);//0x8E
SID130B_write_cmos_sensor(0x1A, 0x00);//0x0A
SID130B_write_cmos_sensor(0x1C, 0x03);                   
SID130B_write_cmos_sensor(0x40, 0x73); //Max Again //anyuan 0x73
SID130B_write_cmos_sensor(0x41, 0x20);
SID130B_write_cmos_sensor(0x42, 0x20);
SID130B_write_cmos_sensor(0x43, 0x00);
SID130B_write_cmos_sensor(0x44, 0x00);
SID130B_write_cmos_sensor(0x45, 0x01);
SID130B_write_cmos_sensor(0x46, 0x0C);
SID130B_write_cmos_sensor(0x47, 0x11);
SID130B_write_cmos_sensor(0x48, 0x15);
SID130B_write_cmos_sensor(0x49, 0x17);
SID130B_write_cmos_sensor(0x4A, 0x1A);
SID130B_write_cmos_sensor(0x4B, 0x1C);
SID130B_write_cmos_sensor(0x4C, 0x1E);
SID130B_write_cmos_sensor(0x4D, 0x1E);
SID130B_write_cmos_sensor(0x4E, 0x0F);
SID130B_write_cmos_sensor(0x4F, 0x09);
SID130B_write_cmos_sensor(0x50, 0x07);
SID130B_write_cmos_sensor(0x51, 0x05);
SID130B_write_cmos_sensor(0x52, 0x04);
SID130B_write_cmos_sensor(0x53, 0x03);
SID130B_write_cmos_sensor(0x54, 0x02);
SID130B_write_cmos_sensor(0x55, 0x01); 

// NORMAL mode AE metering : window setting change
SID130B_write_cmos_sensor(0x60, 0xFF);
SID130B_write_cmos_sensor(0x61, 0xFF);                            
SID130B_write_cmos_sensor(0x62, 0xFF);
SID130B_write_cmos_sensor(0x63, 0xFF);
SID130B_write_cmos_sensor(0x64, 0xFF);
SID130B_write_cmos_sensor(0x65, 0xFF);               
SID130B_write_cmos_sensor(0x66, 0x00);
SID130B_write_cmos_sensor(0x67, 0x50);                            
SID130B_write_cmos_sensor(0x68, 0x50);
SID130B_write_cmos_sensor(0x69, 0x50);
SID130B_write_cmos_sensor(0x6A, 0x50);
SID130B_write_cmos_sensor(0x6B, 0x00);
SID130B_write_cmos_sensor(0x6C, 0x06); 
                        
SID130B_write_cmos_sensor(0x70, 0xc4);
SID130B_write_cmos_sensor(0x73, 0x22);
SID130B_write_cmos_sensor(0x74, 0x07);
SID130B_write_cmos_sensor(0x77, 0xd0);
SID130B_write_cmos_sensor(0x78, 0xd8);                                                            

//AWB Block
SID130B_set_page(0x02); 
SID130B_write_cmos_sensor(0x10, 0xD3);
SID130B_write_cmos_sensor(0x11, 0x11);
SID130B_write_cmos_sensor(0x13, 0x7F);            //# AWB taget Cr
SID130B_write_cmos_sensor(0x14, 0x80);  //AWB taget Cb                          
SID130B_write_cmos_sensor(0x15, 0xEE);
SID130B_write_cmos_sensor(0x16, 0x80);
SID130B_write_cmos_sensor(0x17, 0xD0);
SID130B_write_cmos_sensor(0x18, 0x80);
SID130B_write_cmos_sensor(0x19, 0x98);
SID130B_write_cmos_sensor(0x1A, 0x68);
SID130B_write_cmos_sensor(0x1B, 0x98);
SID130B_write_cmos_sensor(0x1C, 0x68);
SID130B_write_cmos_sensor(0x1D, 0x8C);
SID130B_write_cmos_sensor(0x1E, 0x74);
SID130B_write_cmos_sensor(0x20, 0xF0);
SID130B_write_cmos_sensor(0x21, 0x84);
SID130B_write_cmos_sensor(0x22, 0xB4);
SID130B_write_cmos_sensor(0x23, 0x20);
SID130B_write_cmos_sensor(0x25, 0x20);
SID130B_write_cmos_sensor(0x26, 0x05);
SID130B_write_cmos_sensor(0x27, 0x63); // for outdoor fix gain enter
SID130B_write_cmos_sensor(0x28, 0x63); // for outdoor fix gain quit
SID130B_write_cmos_sensor(0x29, 0xAB); 
SID130B_write_cmos_sensor(0x2A, 0x9D);
SID130B_write_cmos_sensor(0x30, 0x00);
SID130B_write_cmos_sensor(0x31, 0x10); 
SID130B_write_cmos_sensor(0x32, 0x00); 
SID130B_write_cmos_sensor(0x33, 0x10); 
SID130B_write_cmos_sensor(0x34, 0x06); 
SID130B_write_cmos_sensor(0x35, 0x30); 
SID130B_write_cmos_sensor(0x36, 0x04); 
SID130B_write_cmos_sensor(0x37, 0xa0); 
SID130B_write_cmos_sensor(0x40, 0x01); 
SID130B_write_cmos_sensor(0x41, 0x04); 
SID130B_write_cmos_sensor(0x42, 0x08); 
SID130B_write_cmos_sensor(0x43, 0x10); 
SID130B_write_cmos_sensor(0x44, 0x13); 
SID130B_write_cmos_sensor(0x45, 0x6b); 
SID130B_write_cmos_sensor(0x46, 0x82); 

//CMA change  -D65~A
SID130B_write_cmos_sensor(0x53, 0x8a); //AWB R Gain for D30 to D20
SID130B_write_cmos_sensor(0x54, 0xb6); //AWB B Gain for D30 to D20
SID130B_write_cmos_sensor(0x55, 0x8d); //AWB R Gain for D20 to D30
SID130B_write_cmos_sensor(0x56, 0xb5); //AWB B Gain for D20 to D30
SID130B_write_cmos_sensor(0x57, 0xa0); //AWB R Gain for D65 to D30
SID130B_write_cmos_sensor(0x58, 0x88); //AWB B Gain for D65 to D30
SID130B_write_cmos_sensor(0x59, 0xa0); //AWB R Gain for D30 to D65
SID130B_write_cmos_sensor(0x5A, 0xa8); //AWB B Gain for D30 to D65
                
SID130B_write_cmos_sensor(0x64, 0x00); //T1~T2 RGAIN
SID130B_write_cmos_sensor(0x65, 0x00); 
SID130B_write_cmos_sensor(0x66, 0x00); //T3~T4 RGAIN
SID130B_write_cmos_sensor(0x67, 0x00); 
SID130B_write_cmos_sensor(0x68, 0xA5); //T5~T6 RGAIN
SID130B_write_cmos_sensor(0x69, 0xB4); 
SID130B_write_cmos_sensor(0x6A, 0xB3); //T7~T8 RGAIN
SID130B_write_cmos_sensor(0x6B, 0xAC); 
SID130B_write_cmos_sensor(0x6C, 0xB7); //T9~T10 RGAIN
SID130B_write_cmos_sensor(0x6D, 0x98); 
SID130B_write_cmos_sensor(0x6E, 0xBA); //T11~T12 RGAIN
SID130B_write_cmos_sensor(0x6F, 0x90); 
SID130B_write_cmos_sensor(0x70, 0xBF); //T13~T14 RGAIN
SID130B_write_cmos_sensor(0x71, 0x9B); 
SID130B_write_cmos_sensor(0x72, 0xCE); //T15~T16 RGAIN
SID130B_write_cmos_sensor(0x73, 0x8C); 
SID130B_write_cmos_sensor(0x74, 0x7F); //T1 White RGAIN BOT
SID130B_write_cmos_sensor(0x75, 0x8C); //T1 White RGAIN TOP
SID130B_write_cmos_sensor(0x76, 0xAD); //T1 White BGAIN BOT
SID130B_write_cmos_sensor(0x77, 0xBA); //T1 White BGAIN TOP
SID130B_write_cmos_sensor(0x78, 0x8F); //T2 White RGAIN BOT
SID130B_write_cmos_sensor(0x79, 0x9A); //T2 White RGAIN TOP
SID130B_write_cmos_sensor(0x7A, 0xA3); //T2 White BGAIN BOT
SID130B_write_cmos_sensor(0x7B, 0xAC); //T2 White BGAIN TOP
SID130B_write_cmos_sensor(0x7C, 0xA0); //T3 White RGAIN BOT
SID130B_write_cmos_sensor(0x7D, 0xA9); //T3 White RGAIN TOP
SID130B_write_cmos_sensor(0x7E, 0x95); //T3 White BGAIN BOT
SID130B_write_cmos_sensor(0x7F, 0xAC); //T3 White BGAIN TOP
SID130B_write_cmos_sensor(0x80, 0xAD); //T4 White RGAIN BOT
SID130B_write_cmos_sensor(0x81, 0xBC); //T4 White RGAIN TOP
SID130B_write_cmos_sensor(0x82, 0x98); //T4 White BGAIN BOT
SID130B_write_cmos_sensor(0x83, 0xA4); //T4 White BGAIN TOP
SID130B_write_cmos_sensor(0x84, 0x00); //T5 White RGAIN BOT
SID130B_write_cmos_sensor(0x85, 0x00); //T5 White RGAIN TOP   

SID130B_write_cmos_sensor(0x86, 0x00); //T5 White BGAIN BOT
SID130B_write_cmos_sensor(0x87, 0x00); //T5 White BGAIN TOP
SID130B_write_cmos_sensor(0x88, 0xC9); //T6 White RGAIN BOT
SID130B_write_cmos_sensor(0x89, 0xD5); //T6 White RGAIN TOP
SID130B_write_cmos_sensor(0x8A, 0x70); //T6 White BGAIN BOT
SID130B_write_cmos_sensor(0x8B, 0x7B); //T6 White BGAIN TOP
SID130B_write_cmos_sensor(0x8C, 0xD0); //T7 White RGAIN BOT
SID130B_write_cmos_sensor(0x8D, 0xE5); //T7 White RGAIN TOP
SID130B_write_cmos_sensor(0x8E, 0x58); //T7 White BGAIN BOT
SID130B_write_cmos_sensor(0x8F, 0x70); //T7 White BGAIN TOP
SID130B_write_cmos_sensor(0xB4, 0x05); 
SID130B_write_cmos_sensor(0xB5, 0x0F); 
SID130B_write_cmos_sensor(0xB6, 0x06); 
SID130B_write_cmos_sensor(0xB7, 0x06); 
SID130B_write_cmos_sensor(0xB8, 0x40); 
SID130B_write_cmos_sensor(0xB9, 0x10); 
SID130B_write_cmos_sensor(0xBA, 0x06); 

//IDP
SID130B_set_page(0x03); 
SID130B_write_cmos_sensor(0x10, 0xFF);
SID130B_write_cmos_sensor(0x11, 0x1D);//1d
SID130B_write_cmos_sensor(0x12, 0x1D);
SID130B_write_cmos_sensor(0x13, 0xFF);
SID130B_write_cmos_sensor(0x14, 0x00);
SID130B_write_cmos_sensor(0x15, 0xc0);

//DPC
SID130B_write_cmos_sensor(0x30, 0x88); //DPCNRCTRL
SID130B_write_cmos_sensor(0x31, 0x08); //10 100423 DPTHR @ AGAIN = 00 
SID130B_write_cmos_sensor(0x32, 0x08); //0C 100423 DPTHR @ AGAIN = 20
SID130B_write_cmos_sensor(0x33, 0x04); //08 100423 DPTHR @ AGAIN = 40
SID130B_write_cmos_sensor(0x34, 0x00); //04 100423 DPTHR @ AGAIN = 60
SID130B_write_cmos_sensor(0x35, 0x00); //DPTHR @ AGAIN = 80
SID130B_write_cmos_sensor(0x36, 0x44); //DPTHVRNG
SID130B_write_cmos_sensor(0x37, 0x66); //DPNUMBER
SID130B_write_cmos_sensor(0x38, 0x00); //0x00             # NRTHR0 @ AGAIN = 00
SID130B_write_cmos_sensor(0x39, 0x18); //0x0C             # NRTHR1 @ AGAIN = 20
SID130B_write_cmos_sensor(0x3A, 0x40); //0x18             # NRTHR2 @ AGAIN = 40
SID130B_write_cmos_sensor(0x3B, 0x80); //0x30             # NRTHR3 @ AGAIN = 60
SID130B_write_cmos_sensor(0x3C, 0xFF); //80 100423                   NRTHR4 @ AGAIN = 80
SID130B_write_cmos_sensor(0x3D, 0x04); //NRTHVRNG0 @ AGAIN = 00
SID130B_write_cmos_sensor(0x3E, 0x0c); //NRTHVRNG1 @ AGAIN = 20
SID130B_write_cmos_sensor(0x3F, 0x20); //NRTHVRNG2 @ AGAIN = 40
SID130B_write_cmos_sensor(0x40, 0x80); //NRTHVRNG3 @ AGAIN = 60
SID130B_write_cmos_sensor(0x41, 0x80); //NRTHVRNG4 @ AGAIN = 80
SID130B_write_cmos_sensor(0x42, 0x10); //NRTHVRNGMAX
SID130B_write_cmos_sensor(0x43, 0x80); //40 100423 NRTHRWGT
SID130B_write_cmos_sensor(0x44, 0x40); //BASELVL
SID130B_write_cmos_sensor(0x45, 0x05); //06 100423 SHUMAXH
SID130B_write_cmos_sensor(0x46, 0xdc); //40 100423 SHUMAXL
SID130B_write_cmos_sensor(0x47, 0x30); //ILLUMITHDRK

//Shading
SID130B_write_cmos_sensor(0x50, 0x20); //RXSHDL
SID130B_write_cmos_sensor(0x51, 0x32); //RXSHDR
SID130B_write_cmos_sensor(0x52, 0x40); //RXSHDT
SID130B_write_cmos_sensor(0x53, 0x2a); //RXSHDB      
SID130B_write_cmos_sensor(0x54, 0x1b); //GXSHDL
SID130B_write_cmos_sensor(0x55, 0x19); //GXSHDR
SID130B_write_cmos_sensor(0x56, 0x20); //GXSHDT
SID130B_write_cmos_sensor(0x57, 0x15); //GXSHDB      
SID130B_write_cmos_sensor(0x58, 0x1b); //GXSHDL
SID130B_write_cmos_sensor(0x59, 0x19); //GXSHDR
SID130B_write_cmos_sensor(0x5A, 0x20); //GXSHDT
SID130B_write_cmos_sensor(0x5B, 0x15); //GXSHDB      
SID130B_write_cmos_sensor(0x5C, 0x33); //BXSHDL
SID130B_write_cmos_sensor(0x5D, 0x42); //BXSHDR
SID130B_write_cmos_sensor(0x5E, 0x49); //BXSHDT
SID130B_write_cmos_sensor(0x5F, 0x30); //BXSHDB      
SID130B_write_cmos_sensor(0x60, 0x32); //#SHDCNTH(X/Y)
SID130B_write_cmos_sensor(0x61, 0x20); //#SHDCNTX
SID130B_write_cmos_sensor(0x62, 0x67); //#SHDCNTY 
SID130B_write_cmos_sensor(0x63, 0x8A); 
SID130B_write_cmos_sensor(0x66, 0x76);            
SID130B_write_cmos_sensor(0x6B, 0x01);
SID130B_write_cmos_sensor(0x6C, 0x22);
SID130B_write_cmos_sensor(0x6D, 0x23);
SID130B_write_cmos_sensor(0x6E, 0x55);
SID130B_write_cmos_sensor(0x6F, 0x77);
SID130B_write_cmos_sensor(0x70, 0x65);
SID130B_write_cmos_sensor(0x71, 0x01);
SID130B_write_cmos_sensor(0x72, 0x23);
SID130B_write_cmos_sensor(0x73, 0x33);
SID130B_write_cmos_sensor(0x74, 0x45);
SID130B_write_cmos_sensor(0x75, 0x55);
SID130B_write_cmos_sensor(0x76, 0x55);
SID130B_write_cmos_sensor(0x77, 0x01);
SID130B_write_cmos_sensor(0x78, 0x23);
SID130B_write_cmos_sensor(0x79, 0x33);
SID130B_write_cmos_sensor(0x7A, 0x45);
SID130B_write_cmos_sensor(0x7B, 0x55);
SID130B_write_cmos_sensor(0x7C, 0x55);
SID130B_write_cmos_sensor(0x7D, 0x00);
SID130B_write_cmos_sensor(0x7E, 0x01);
SID130B_write_cmos_sensor(0x7F, 0x11);
SID130B_write_cmos_sensor(0x80, 0x33);
SID130B_write_cmos_sensor(0x81, 0x33);
SID130B_write_cmos_sensor(0x82, 0x22);
SID130B_write_cmos_sensor(0x83, 0x18);//14 100423
SID130B_write_cmos_sensor(0x84, 0x0f);

//SID130B_write_cmos_sensor(0x94, 0x06);
//SID130B_write_cmos_sensor(0x95, 0x40);
//SID130B_write_cmos_sensor(0x96, 0x04);
//SID130B_write_cmos_sensor(0x97, 0xb0);

//Interpolation
SID130B_write_cmos_sensor(0xA0, 0x2F);
SID130B_write_cmos_sensor(0xA1, 0x04);
SID130B_write_cmos_sensor(0xA2, 0xB7);
SID130B_write_cmos_sensor(0xA3, 0xB7);
SID130B_write_cmos_sensor(0xA4, 0x08);
SID130B_write_cmos_sensor(0xA5, 0xFF);
SID130B_write_cmos_sensor(0xA6, 0x00);
SID130B_write_cmos_sensor(0xA7, 0xFF);
SID130B_write_cmos_sensor(0xA8, 0x00);
SID130B_write_cmos_sensor(0xA9, 0x00);
SID130B_write_cmos_sensor(0xAA, 0x00);
SID130B_write_cmos_sensor(0xAB, 0x00);
SID130B_write_cmos_sensor(0xAC, 0x60);
SID130B_write_cmos_sensor(0xAD, 0x18);
SID130B_write_cmos_sensor(0xAE, 0x10);
SID130B_write_cmos_sensor(0xAF, 0x20);
SID130B_write_cmos_sensor(0xB0, 0x08);
SID130B_write_cmos_sensor(0xB1, 0x00);

//Color Matrix for D65
SID130B_write_cmos_sensor(0xC0, 0xaF);// CMASB D20 or D30 or Dark Condition Color Matrix Selection
SID130B_write_cmos_sensor(0xC1, 0x66);
SID130B_write_cmos_sensor(0xC2, 0xd4);
SID130B_write_cmos_sensor(0xC3, 0x06);
SID130B_write_cmos_sensor(0xC4, 0xF0);
SID130B_write_cmos_sensor(0xC5, 0x5A);
SID130B_write_cmos_sensor(0xC6, 0xF6);
SID130B_write_cmos_sensor(0xC7, 0xF9);
SID130B_write_cmos_sensor(0xC8, 0xBF);
SID130B_write_cmos_sensor(0xC9, 0x88);
SID130B_write_cmos_sensor(0xCA, 0x00);
SID130B_write_cmos_sensor(0xCB, 0x00);
SID130B_write_cmos_sensor(0xCC, 0x00);
SID130B_write_cmos_sensor(0xCD, 0x00);
SID130B_write_cmos_sensor(0xCE, 0x00);

//Color Matrix for CWF
SID130B_write_cmos_sensor(0xD0, 0x2F);
SID130B_write_cmos_sensor(0xD1, 0x71);
SID130B_write_cmos_sensor(0xD2, 0xb6);
SID130B_write_cmos_sensor(0xD3, 0x19);
SID130B_write_cmos_sensor(0xD4, 0xE8);
SID130B_write_cmos_sensor(0xD5, 0x5a);
SID130B_write_cmos_sensor(0xD6, 0xFe);
SID130B_write_cmos_sensor(0xD7, 0xe8);
SID130B_write_cmos_sensor(0xD8, 0xae);
SID130B_write_cmos_sensor(0xD9, 0xaa);
SID130B_write_cmos_sensor(0xDA, 0x00);
SID130B_write_cmos_sensor(0xDB, 0x00);
SID130B_write_cmos_sensor(0xDC, 0x00);
SID130B_write_cmos_sensor(0xDD, 0x00);
SID130B_write_cmos_sensor(0xDE, 0x00);

//Color Matrix for A
SID130B_write_cmos_sensor(0xE0, 0x2F);
SID130B_write_cmos_sensor(0xE1, 0x5C);
SID130B_write_cmos_sensor(0xE2, 0xDD);
SID130B_write_cmos_sensor(0xE3, 0x06);
SID130B_write_cmos_sensor(0xE4, 0xE0);
SID130B_write_cmos_sensor(0xE5, 0x69);
SID130B_write_cmos_sensor(0xE6, 0xF6);
SID130B_write_cmos_sensor(0xE7, 0xE5);
SID130B_write_cmos_sensor(0xE8, 0xAB);
SID130B_write_cmos_sensor(0xE9, 0xAE);
SID130B_write_cmos_sensor(0xEA, 0x6A);
SID130B_write_cmos_sensor(0xEB, 0x01);
SID130B_write_cmos_sensor(0xEC, 0x2D);
SID130B_write_cmos_sensor(0xED, 0xEE);
SID130B_write_cmos_sensor(0xEE, 0x04);

//IDP 2
SID130B_set_page(0x04);             
//Gamma - R
SID130B_write_cmos_sensor(0x10, 0x00);                
SID130B_write_cmos_sensor(0x11, 0x05);
SID130B_write_cmos_sensor(0x12, 0x0c);
SID130B_write_cmos_sensor(0x13, 0x1d);
SID130B_write_cmos_sensor(0x14, 0x45);
SID130B_write_cmos_sensor(0x15, 0x64);
SID130B_write_cmos_sensor(0x16, 0x7c);
SID130B_write_cmos_sensor(0x17, 0x8e);
SID130B_write_cmos_sensor(0x18, 0xa0);
SID130B_write_cmos_sensor(0x19, 0xae);
SID130B_write_cmos_sensor(0x1A, 0xbb);
SID130B_write_cmos_sensor(0x1B, 0xd0);
SID130B_write_cmos_sensor(0x1C, 0xe2);
SID130B_write_cmos_sensor(0x1D, 0xf2);
SID130B_write_cmos_sensor(0x1E, 0xf9);
SID130B_write_cmos_sensor(0x1F, 0xFF);
                                                                                                                
//Gamma - G                                      
SID130B_write_cmos_sensor(0x20, 0x00);
SID130B_write_cmos_sensor(0x21, 0x05);
SID130B_write_cmos_sensor(0x22, 0x0c);
SID130B_write_cmos_sensor(0x23, 0x1d);
SID130B_write_cmos_sensor(0x24, 0x45);
SID130B_write_cmos_sensor(0x25, 0x64);
SID130B_write_cmos_sensor(0x26, 0x7c);
SID130B_write_cmos_sensor(0x27, 0x8e);
SID130B_write_cmos_sensor(0x28, 0xa0);
SID130B_write_cmos_sensor(0x29, 0xae);
SID130B_write_cmos_sensor(0x2A, 0xbb);
SID130B_write_cmos_sensor(0x2B, 0xd0);
SID130B_write_cmos_sensor(0x2C, 0xe2);
SID130B_write_cmos_sensor(0x2D, 0xf2);
SID130B_write_cmos_sensor(0x2E, 0xf9);
SID130B_write_cmos_sensor(0x2F, 0xFF);
                                                                                               
//Gamma - B                                      
SID130B_write_cmos_sensor(0x30, 0x00);
SID130B_write_cmos_sensor(0x31, 0x05);
SID130B_write_cmos_sensor(0x32, 0x0c);
SID130B_write_cmos_sensor(0x33, 0x1d);
SID130B_write_cmos_sensor(0x34, 0x45);
SID130B_write_cmos_sensor(0x35, 0x64);
SID130B_write_cmos_sensor(0x36, 0x7c);
SID130B_write_cmos_sensor(0x37, 0x8e);
SID130B_write_cmos_sensor(0x38, 0xa0);
SID130B_write_cmos_sensor(0x39, 0xae);
SID130B_write_cmos_sensor(0x3A, 0xbb);
SID130B_write_cmos_sensor(0x3B, 0xd0);
SID130B_write_cmos_sensor(0x3C, 0xe2);
SID130B_write_cmos_sensor(0x3D, 0xf2);
SID130B_write_cmos_sensor(0x3E, 0xf9);
SID130B_write_cmos_sensor(0x3F, 0xFF);
                                                                                                
//CSC                                                                    
SID130B_write_cmos_sensor(0x60, 0x33);
SID130B_write_cmos_sensor(0x61, 0x20);
SID130B_write_cmos_sensor(0x62, 0xE4);
SID130B_write_cmos_sensor(0x63, 0xFA);
SID130B_write_cmos_sensor(0x64, 0x13);
SID130B_write_cmos_sensor(0x65, 0x25);
SID130B_write_cmos_sensor(0x66, 0x07);
SID130B_write_cmos_sensor(0x67, 0xF5);
SID130B_write_cmos_sensor(0x68, 0xEA);
SID130B_write_cmos_sensor(0x69, 0x20);
SID130B_write_cmos_sensor(0x6A, 0xC8);
SID130B_write_cmos_sensor(0x6B, 0xC4);
SID130B_write_cmos_sensor(0x6C, 0x84);
SID130B_write_cmos_sensor(0x6D, 0x04);
SID130B_write_cmos_sensor(0x6E, 0x0C);
SID130B_write_cmos_sensor(0x6F, 0x00);
                                                                                                
//Edge                                                                  
SID130B_write_cmos_sensor(0x80, 0xA2);
SID130B_write_cmos_sensor(0x81, 0x10);
SID130B_write_cmos_sensor(0x82, 0x14);
SID130B_write_cmos_sensor(0x83, 0x04);
SID130B_write_cmos_sensor(0x84, 0x18);
SID130B_write_cmos_sensor(0x85, 0x06);
SID130B_write_cmos_sensor(0x86, 0x00);
SID130B_write_cmos_sensor(0x87, 0x04);
SID130B_write_cmos_sensor(0x88, 0x18);
SID130B_write_cmos_sensor(0x89, 0x06);
SID130B_write_cmos_sensor(0x8a, 0x00);
SID130B_write_cmos_sensor(0x8b, 0x24);
SID130B_write_cmos_sensor(0x8c, 0x24);
SID130B_write_cmos_sensor(0X90, 0x16);
SID130B_write_cmos_sensor(0x91, 0x03);
SID130B_write_cmos_sensor(0x93, 0xE0);

//Cr/Cb Coring
SID130B_write_cmos_sensor(0x94, 0x1b);//1b
SID130B_write_cmos_sensor(0x95, 0x25);//1b
SID130B_write_cmos_sensor(0x96, 0x4C);
SID130B_write_cmos_sensor(0x97, 0x76); 
SID130B_write_cmos_sensor(0x98, 0x30);//100423
SID130B_write_cmos_sensor(0x9A, 0xF5);
SID130B_write_cmos_sensor(0xA1, 0x08);
SID130B_write_cmos_sensor(0xA2, 0x10);
SID130B_write_cmos_sensor(0xA3, 0x20);//16 100423
SID130B_write_cmos_sensor(0xA4, 0x40);//20 100423
SID130B_write_cmos_sensor(0xA5, 0xff);//30 100423
SID130B_write_cmos_sensor(0xA6, 0xA0);
SID130B_write_cmos_sensor(0xA7, 0x05);//09 100423
SID130B_write_cmos_sensor(0xA8, 0xdc);//60 100423           
SID130B_write_cmos_sensor(0xA9, 0x20);
SID130B_write_cmos_sensor(0xAA, 0x60);          
SID130B_write_cmos_sensor(0xAC, 0xFF);
SID130B_write_cmos_sensor(0xAD, 0x09);
SID130B_write_cmos_sensor(0xAE, 0x96);
SID130B_write_cmos_sensor(0xAF, 0x18);
SID130B_write_cmos_sensor(0xB2, 0x30); //50 100423 color suppression start level            //0x40
SID130B_write_cmos_sensor(0xB3, 0x14); //[7:4]color suppression slope //0x33                                                                                                                                                    //[3:0]color suppression end gain
//SID130B_write_cmos_sensor(0xB6, 0x08); //Brightness Control

//Color Saturation
SID130B_write_cmos_sensor(0xBC, 0x15); //0x14
SID130B_write_cmos_sensor(0xBD, 0x15);
SID130B_write_cmos_sensor(0xBE, 0x16);
SID130B_write_cmos_sensor(0xBF, 0x16); 
SID130B_write_cmos_sensor(0xc0, 0x10);
SID130B_write_cmos_sensor(0xc1, 0x10);
SID130B_write_cmos_sensor(0xc2, 0x14);
SID130B_write_cmos_sensor(0xc3, 0x14);
SID130B_write_cmos_sensor(0xc4, 0x14);
SID130B_write_cmos_sensor(0xc5, 0x14);
SID130B_write_cmos_sensor(0xc6, 0x01);
SID130B_write_cmos_sensor(0xc7, 0x01);
SID130B_write_cmos_sensor(0xc8, 0x01);
SID130B_write_cmos_sensor(0xc9, 0x01);
SID130B_write_cmos_sensor(0xca, 0x01);
SID130B_write_cmos_sensor(0xcb, 0x01);
SID130B_write_cmos_sensor(0xcc, 0x04);
SID130B_write_cmos_sensor(0xcd, 0x3f);
SID130B_write_cmos_sensor(0xce, 0x01);            
                                                                                               
//IDP 3                                                  
SID130B_set_page(0x05); 
                                                                                               
//Memory                                                           
SID130B_write_cmos_sensor(0x40, 0x15);
SID130B_write_cmos_sensor(0x41, 0x28);
SID130B_write_cmos_sensor(0x42, 0x04);
SID130B_write_cmos_sensor(0x43, 0x15);
SID130B_write_cmos_sensor(0x44, 0x28);
SID130B_write_cmos_sensor(0x45, 0x04);
SID130B_write_cmos_sensor(0x46, 0x15);
SID130B_write_cmos_sensor(0x47, 0x28);
SID130B_write_cmos_sensor(0x48, 0x04);
                                                                                               
//Knee                                                                 
SID130B_write_cmos_sensor(0x90, 0xCA); //
SID130B_write_cmos_sensor(0x91, 0x81); //knee function selection/knee point H
SID130B_write_cmos_sensor(0x92, 0x00); //knee point L
SID130B_write_cmos_sensor(0x93, 0x50); //Knee gain
SID130B_write_cmos_sensor(0x94, 0x41); //[6:4]knee start H/[2:0]Knee END H
SID130B_write_cmos_sensor(0x95, 0x7E); //knee start L
SID130B_write_cmos_sensor(0x96, 0x48); //knee END L
                                                                                                
//ADG                                                                   
SID130B_write_cmos_sensor(0x99, 0xC0);
SID130B_write_cmos_sensor(0xA0, 0x10);
SID130B_write_cmos_sensor(0xA1, 0x22);
SID130B_write_cmos_sensor(0xA2, 0x36);
SID130B_write_cmos_sensor(0xA3, 0x49);
SID130B_write_cmos_sensor(0xA4, 0x5D);
SID130B_write_cmos_sensor(0xA5, 0x70);
SID130B_write_cmos_sensor(0xA6, 0x82);
SID130B_write_cmos_sensor(0xA7, 0x94);
SID130B_write_cmos_sensor(0xA8, 0xA5);
SID130B_write_cmos_sensor(0xA9, 0xB5);
SID130B_write_cmos_sensor(0xAA, 0xC3);
SID130B_write_cmos_sensor(0xAB, 0xD1);
SID130B_write_cmos_sensor(0xAC, 0xDE);
SID130B_write_cmos_sensor(0xAD, 0xEA);
SID130B_write_cmos_sensor(0xAE, 0xF5);
SID130B_write_cmos_sensor(0xAF, 0xFF);
                                                                                                
//YXGMA                                                             
SID130B_write_cmos_sensor(0xB0, 0xc0); //YGMACTRL
SID130B_write_cmos_sensor(0xB1, 0x04); //YGMASLOP
SID130B_write_cmos_sensor(0xB8, 0x0f); //DRKTHR1
SID130B_write_cmos_sensor(0xB9, 0x10); //DRKTHR2
//SID130B_write_cmos_sensor(0xBA,    0x38); //DRKTHR3
//SID130B_write_cmos_sensor(0xBB,    0x39); //DRKTHR4
SID130B_write_cmos_sensor(0xC0, 0x03);
SID130B_write_cmos_sensor(0xC1, 0x0E);
SID130B_write_cmos_sensor(0xC2, 0x16);
SID130B_write_cmos_sensor(0xC3, 0x24);
SID130B_write_cmos_sensor(0xC4, 0x3F);
SID130B_write_cmos_sensor(0xC5, 0x56);
SID130B_write_cmos_sensor(0xC6, 0x6A);
SID130B_write_cmos_sensor(0xC7, 0x7C);
SID130B_write_cmos_sensor(0xC8, 0x8C);
SID130B_write_cmos_sensor(0xC9, 0x98);
SID130B_write_cmos_sensor(0xCA, 0xA2);
SID130B_write_cmos_sensor(0xCB, 0xB8);
SID130B_write_cmos_sensor(0xCC, 0xCD);
SID130B_write_cmos_sensor(0xCD, 0xE2);
SID130B_write_cmos_sensor(0xCE, 0xF0);
SID130B_write_cmos_sensor(0xCF, 0xFF);

SID130B_write_cmos_sensor(0xe0, 0x04);
SID130B_write_cmos_sensor(0xe1, 0x88);
SID130B_write_cmos_sensor(0xe2, 0x09);
SID130B_write_cmos_sensor(0xe3, 0x0c);
SID130B_write_cmos_sensor(0xe4, 0x11);
SID130B_write_cmos_sensor(0xe5, 0x16);

SID130B_write_cmos_sensor(0xe6, 0x1b);
SID130B_write_cmos_sensor(0xe7, 0x24);
SID130B_write_cmos_sensor(0xe8, 0x30);
SID130B_write_cmos_sensor(0xe9, 0x3e);           

//Sensor on
SID130B_set_page(0x00);
SID130B_write_cmos_sensor(0x03, (sensor_current << 4) | 0x5);

SID130B_set_page(0x01); 
SID130B_write_cmos_sensor(0x10, 0xc4);


#endif
} /* SID130B_YUV_sensor_initial_setting */





/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
UINT32 SID130BOpen(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

	SENSORDB("zhijie SID130BOpen ");

	//  Read sensor ID to adjust I2C is OK?
    SID130B_write_cmos_sensor(0x00,0x00);
	for(i=0;i<3;i++)
	{
		sensor_id = SID130B_read_cmos_sensor(0x01);
		SENSORDB("IV120D Sensor Read ID %d\n",sensor_id);
		if((sensor_id != SID130B_SENSOR_ID) && (i == 2))
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
	//iowrite32(0xe<<12, 0xF0001500);
	
SID130B_mclk_div_Pre = 1; 
	SID130B_mclk_div_Cap = 1;
    SID130B_Video_Mode = FALSE;
    SID130B_Mode_change = FALSE;        
    SID130B_g_bNightMode = FALSE;
    SID130B_g_iBanding = AE_FLICKER_MODE_50HZ;
    SID130B_Control = 0x00;
	SID130B_HVMirror = IMAGE_NORMAL;
	SID130B_PV_Dummy_Pixel = 0x42;
	SID130B_Min_Fps_Normal = 10 * SID130B_FRAME_RATE_UNIT;
	SID130B_Min_Fps_Night = SID130B_Min_Fps_Normal >> 1;
	
    SID130B_cal_fps();  
	RETAILMSG(1, (TEXT("SID130B Sensor Read ID OK \r\n")));

    /*9. Apply sensor initail setting*/
     SID130B_YUV_sensor_initial_setting();
       

	return ERROR_NONE;
}	/* SID130BOpen() */

UINT32 SID130BGetSensorID(UINT32 *sensorID) 
{
	volatile signed char i;

	SENSORDB("zhijie SID130BGetSensorID ");

	//	Read sensor ID to adjust I2C is OK?
	SID130B_write_cmos_sensor(0x00,0x00);
	for(i=0;i<3;i++)
	{
		*sensorID = SID130B_read_cmos_sensor(0x01);
		SENSORDB("IV120D Sensor Read ID %d\n",*sensorID);
		if((*sensorID != SID130B_SENSOR_ID) && (i == 2))
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
	return ERROR_NONE;
}


UINT32 SID130BClose(void)
{

	return ERROR_NONE;
}	/* SID130BClose() */

UINT32 SID130BPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    //kal_uint8 iTemp = 0;
    kal_uint16 iStartX = 0, iStartY = 2;
	kal_bool  SID130B_Mode_temp = FALSE;

	SENSORDB("[SID130BYUV]SID130BPreview zhijie1111\n");

	/* config TG for camera preview mode */ 
	SID130B_Mode_temp = FALSE;
	SID130B_g_bNightMode = FALSE;
	SID130B_Min_Fps_Normal = 10 * SID130B_FRAME_RATE_UNIT;//normal 10fps night 5 fps 
	SID130B_Min_Fps_Night = SID130B_Min_Fps_Normal >> 1; 
	/* select pclk = mclk/2, Vclk = mclk*/
	SID130B_Control = SID130B_Control & 0xEF | 0x00;  
	SID130B_FixFrameRate(FALSE);

	if ( SID130B_Mode_temp !=  SID130B_Video_Mode)
	{
		 SID130B_Video_Mode =  SID130B_Mode_temp;
		 SID130B_Mode_change = TRUE;	   
	}
	//SID130B_Dummy_Pixel = 0x1c;
    
    SID130B_SID130B_HVMirror(sensor_config_data->SensorImageMirror);
	SID130B_set_outdoor_fix_gain_conditon(SID130B_outdoor_condition);
    //use group_B register
	SID130B_set_page(0);
	/*SID130B_Control [7:6]fix frame rate
	 [4] Group A or Group B
	 [3:2] PCLK divider
	 [1:0] Mirror and flip
	*/
	SID130B_write_cmos_sensor(0x04,SID130B_Control);
	
	
    image_window->GrabStartX= iStartX;
    image_window->GrabStartY = iStartY;
    image_window->ExposureWindowWidth = SID130B_IMAGE_SENSOR_PV_WIDTH -8;
    image_window->ExposureWindowHeight = SID130B_IMAGE_SENSOR_PV_HEIGHT - 6;



    return TRUE; 
}	/* SID130BPreview() */

UINT32 SID130BCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint8 iStartX = 0, iStartY = 2;    
		kal_uint16 iDummy_pixel = 0;

	SENSORDB("Camera Capture\r\n");
	if((image_window->ImageTargetWidth <= SID130B_IMAGE_SENSOR_PV_WIDTH )&&
		image_window->ImageTargetHeight <= SID130B_IMAGE_SENSOR_PV_HEIGHT)
	{
		image_window->GrabStartX = iStartX + 8;
		image_window->GrabStartY = iStartY +7;
		image_window->ExposureWindowWidth = SID130B_IMAGE_SENSOR_PV_WIDTH - 8;
		image_window->ExposureWindowHeight = SID130B_IMAGE_SENSOR_PV_HEIGHT - 6;
	}
	else
	{
		// Capture frame rate 9.23 fps
		
		SENSORDB("Camera Capture full size 1111\r\n");
		//iDummy_pixel = 0x158;
		
		iDummy_pixel = 0x02;
		if(SID130B_zoom_factor >= 3 && SID130B_zoom_factor <= 6)
			iDummy_pixel = 0x4E4;
		else if (SID130B_zoom_factor > 6)
			iDummy_pixel = 0x7D2;
		
		SID130B_Config_Cap_Blank(iDummy_pixel,0x00);//SID130B_Config_Cap_Blank(iDummy_pixel,0x3D);
		//use group_A register
		SID130B_set_page(0);
		SID130B_Control = SID130B_Control |0x10; 
		SID130B_write_cmos_sensor(0x04,SID130B_Control);
		Sleep(30);

	
		image_window->GrabStartX = iStartX + 8;
		image_window->GrabStartY = iStartY + 7;
		image_window->ExposureWindowWidth = SID130B_IMAGE_SENSOR_FULL_WIDTH - 8;
		image_window->ExposureWindowHeight = SID130B_IMAGE_SENSOR_FULL_HEIGHT - 6;
	}
		// copy sensor_config_data
		memcpy(&SID130BSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
		return ERROR_NONE;

}	/* SID130BCapture() */

UINT32 SID130BGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	SENSORDB("[SID130BYUV]SID130BGetResolution zhijie1111111\n");

	pSensorResolution->SensorFullWidth=SID130B_IMAGE_SENSOR_FULL_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;  //modify by yanxu
	pSensorResolution->SensorFullHeight=SID130B_IMAGE_SENSOR_FULL_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	pSensorResolution->SensorPreviewWidth=SID130B_IMAGE_SENSOR_PV_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
	pSensorResolution->SensorPreviewHeight=SID130B_IMAGE_SENSOR_PV_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	SENSORDB("fullwidth=%d fullheight=%d,previewwidth=%d,previewheight=%d\n",pSensorResolution->SensorFullWidth,
	pSensorResolution->SensorFullHeight,pSensorResolution->SensorPreviewWidth,pSensorResolution->SensorPreviewHeight	);

	return ERROR_NONE;
}	/* SID130BGetResolution() */

UINT32 SID130BGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	SENSORDB("[SID130BYUV]SID130BGetInfo zhijie111111 ScenarioId=%d\n",ScenarioId);

	pSensorInfo->SensorPreviewResolutionX=SID130B_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY=SID130B_IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX=SID130B_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=SID130B_IMAGE_SENSOR_FULL_HEIGHT;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YCbYCr;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	/*??? */
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;      //HSYNC HIGH valid
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;      //VSYNC LOW valid
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].BinningEnable=TRUE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].BinningEnable=TRUE;

	pSensorInfo->CaptureDelayFrame = 2; 
	pSensorInfo->PreviewDelayFrame = 0; 
	pSensorInfo->VideoDelayFrame = 0; 		
	pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA; 		
   
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			pSensorInfo->SensorClockFreq=26;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = 0; 
			pSensorInfo->SensorGrabStartY = 0;          
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			pSensorInfo->SensorClockFreq=26;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = 0; 
			pSensorInfo->SensorGrabStartY = 0;       
		break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = 0; 
			pSensorInfo->SensorGrabStartY = 0;       
		break;
	}
	//SID130B_PixelClockDivider=pSensorInfo->SensorPixelClockCount;
	memcpy(pSensorConfigData, &SID130BSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* SID130BGetInfo() */


UINT32 SID130BControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	SENSORDB("[SID130BYUV]SID130BControl zhijie111111\n");

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			SID130BPreview(pImageWindow, pSensorConfigData);
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			SID130BCapture(pImageWindow, pSensorConfigData);
		break;
		default:
		    break; 
	}
	return TRUE;
}	/* SID130BControl() */

/* [TC] YUV sensor */	

BOOL SID130B_set_param_wb(UINT16 para)
{
	SID130B_set_page(2);
    switch (para)
    {
        case AWB_MODE_AUTO:   // enable AWB
            // CAM_WB_AUTO mode should not update R/G/B gains
            SID130B_write_cmos_sensor(0x10, 0xD3);
            break;
                
        case AWB_MODE_DAYLIGHT:
            SID130B_write_cmos_sensor(0x10, 0x00);  // disable AWB
            SID130B_write_cmos_sensor(0x50, 0xC2);
            SID130B_write_cmos_sensor(0x51, 0x9E);
            break;
        
        case AWB_MODE_INCANDESCENT:
            SID130B_write_cmos_sensor(0x10, 0x00);  // disable AWB
            SID130B_write_cmos_sensor(0x50, 0x98);
            SID130B_write_cmos_sensor(0x51, 0xC8);
            break;
        
        case AWB_MODE_FLUORESCENT:
            SID130B_write_cmos_sensor(0x10, 0x00);  // disable AWB
            SID130B_write_cmos_sensor(0x50, 0xAA);
            SID130B_write_cmos_sensor(0x51, 0xBE);
            break;
        
        case AWB_MODE_TUNGSTEN:
            SID130B_write_cmos_sensor(0x10, 0x00);  // disable AWB
            SID130B_write_cmos_sensor(0x60, 0x90);
            SID130B_write_cmos_sensor(0x61, 0xC0);
            break;
        
        default:
            return FALSE;
    }
    return TRUE;
} /* SID130B_set_param_wb */

BOOL SID130B_set_param_effect(UINT16 para)
{
	SID130B_set_page(4);
    switch (para)
    {
        case MEFFECT_OFF:
            SID130B_write_cmos_sensor(0xD9, 0x00);
            break;

        
        case MEFFECT_SEPIA:
            SID130B_write_cmos_sensor(0xD9, 0x80);
            SID130B_write_cmos_sensor(0xDA, 0x60);
            SID130B_write_cmos_sensor(0xDB, 0xA0);
            break;

        case MEFFECT_SEPIAGREEN:
            SID130B_write_cmos_sensor(0xD9, 0x80);
            SID130B_write_cmos_sensor(0xDA, 0x50);
            SID130B_write_cmos_sensor(0xDB, 0x50);
            break;

        case MEFFECT_SEPIABLUE:
            SID130B_write_cmos_sensor(0xD9, 0x80);
            SID130B_write_cmos_sensor(0xDA, 0xC0);
            SID130B_write_cmos_sensor(0xDB, 0x60);
            break;
        
        case MEFFECT_NEGATIVE:
            SID130B_write_cmos_sensor(0xD9, 0x10);
            break;
            
        case MEFFECT_WHITEBOARD:
            SID130B_write_cmos_sensor(0xD9, 0x20);
            break;

        case MEFFECT_SOLARIZE:
            SID130B_write_cmos_sensor(0xD9, 0x08);
            break;
        
        case MEFFECT_AQUA:
            SID130B_write_cmos_sensor(0xD9, 0x04);
            break;
             
        default:
            return FALSE;
    }
    return TRUE;
} /* SID130B_set_param_effect */

BOOL SID130B_set_param_banding(UINT16 para)
{
		if(SID130B_g_iBanding == para)
			return TRUE;
		
		SID130B_g_iBanding = para;
		SID130B_PV_Set_Shutter_Step();
		SID130B_Set_Frame_Count();
		
		return TRUE;
} /* SID130B_set_param_banding */

BOOL SID130B_set_param_exposure(UINT16 para)
{
	SID130B_set_page(1);
	
	SENSORDB("SID130B_set_param_exposure para =%d\n",para);
    switch (para)
    {
        case AE_EV_COMP_n13:    // -4 EV
            SID130B_write_cmos_sensor(0x12, 0x50);
            SID130B_write_cmos_sensor(0x13, 0x50);
            SID130B_write_cmos_sensor(0x14, 0x50);
            break;
        
        case AE_EV_COMP_n10:    // -3 EV
            SID130B_write_cmos_sensor(0x12, 0x60);
            SID130B_write_cmos_sensor(0x13, 0x60);
            SID130B_write_cmos_sensor(0x14, 0x60);
            break;
        
        case AE_EV_COMP_n07:    // -2 EV
            SID130B_write_cmos_sensor(0x12, 0x70);
            SID130B_write_cmos_sensor(0x13, 0x70);
            SID130B_write_cmos_sensor(0x14, 0x70);
            break;
        
        case AE_EV_COMP_n03:    // -1 EV
            SID130B_write_cmos_sensor(0x12, 0x80);
            SID130B_write_cmos_sensor(0x13, 0x80);
            SID130B_write_cmos_sensor(0x14, 0x80);
            break;
        
        case AE_EV_COMP_00:   // +0 EV
            SID130B_write_cmos_sensor(0x12, 0x78); //0x88
            SID130B_write_cmos_sensor(0x13, 0x78);
            SID130B_write_cmos_sensor(0x14, 0x78);
            break;
        
        case AE_EV_COMP_03:    // +1 EV
            SID130B_write_cmos_sensor(0x12, 0xA0);
            SID130B_write_cmos_sensor(0x13, 0xA0);
            SID130B_write_cmos_sensor(0x14, 0xA0);	
            break;
        
        case AE_EV_COMP_07:    // +2 EV
            SID130B_write_cmos_sensor(0x12, 0xB0);
            SID130B_write_cmos_sensor(0x13, 0xB0);
            SID130B_write_cmos_sensor(0x14, 0xB0);
            break;
        
        case AE_EV_COMP_10:    // +3 EV
            SID130B_write_cmos_sensor(0x12, 0xC0);
            SID130B_write_cmos_sensor(0x13, 0xC0);
            SID130B_write_cmos_sensor(0x14, 0xC0);
            break;
        
        case AE_EV_COMP_13:    // +4 EV
            SID130B_write_cmos_sensor(0x12, 0xD0);
            SID130B_write_cmos_sensor(0x13, 0xD0);
            SID130B_write_cmos_sensor(0x14, 0xD0);
            break;
        
        default:
            return FALSE;    
    }

    return TRUE;	
} /* SID130B_set_param_exposure */



UINT32 SID130BYUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
    SENSORDB("SID130BYUVSensorSetting \n");

	switch (iCmd) {
	case FID_SCENE_MODE:	    
	    if (iPara == SCENE_MODE_OFF)
	    {
	    
		SENSORDB("SID130B_NightMode = FALSE \n");
	        SID130B_NightMode(FALSE); 
	    }
	    else if (iPara == SCENE_MODE_NIGHTSCENE)
	    {
			SENSORDB("SID130B_NightMode = TRUE \n");
               SID130B_NightMode(TRUE); 
	    }	    
	    break; 	    
	case FID_AWB_MODE:
		
		SENSORDB("SID130B_set_param_wb \n");
           SID130B_set_param_wb(iPara);
	break;
	case FID_COLOR_EFFECT:
		
		SENSORDB("SID130B_set_param_effect \n");
           SID130B_set_param_effect(iPara);
	break;
	case FID_AE_EV:
		
		SENSORDB("SID130B_set_param_exposure \n");
           SID130B_set_param_exposure(iPara);
	break;
	case FID_AE_FLICKER:
		
		SENSORDB("SID130B_set_param_banding \n");
           SID130B_set_param_banding(iPara);
	break;
	
	case FID_ZOOM_FACTOR:
		
		SENSORDB("SID130B_set_param_banding FID_ZOOM_FACTOR=%d\n",iPara);
		SID130B_zoom_factor = iPara;
	break;
	default:
	break;
	}
	return TRUE;
}   /* SID130BYUVSensorSetting */

UINT32 SID130BYUVSetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("SetVideoMode %d\n",u2FrameRate);
	SID130B_Video_Mode =TRUE;

	SID130B_FixFrameRate(TRUE);  

    if (u2FrameRate == 30)
    {
    	
		SID130B_Min_Fps_Video = 30 *	SID130B_FRAME_RATE_UNIT;//normal 30fps night 15 fps 
    }
    else if (u2FrameRate == 15)       
    {
                
		SID130B_Min_Fps_Video =  15  *	SID130B_FRAME_RATE_UNIT;; 
    }
    else 
    {
        printk("Wrong frame rate setting \n");
    }   
	SID130B_set_page(4);
	SID130B_write_cmos_sensor(0xb6,0x08); //Brightness Control 0x11
	
	SID130B_cal_fps();	  
    SID130B_Set_Frame_Count();     
    return TRUE;
}

UINT32 SID130BFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
       UINT16 u2Temp = 0; 
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

#if WINMO_USE	
	PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo=(PMSDK_FEATURE_INFO_STRUCT) pFeaturePara;
#endif 
//	SENSORDB("[SID130BYUV]SID130BFeatureControl =%d\n",FeatureId);

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=SID130B_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=SID130B_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++=SID130B_PV_PERIOD_PIXEL_NUMS;//+SID130B_PV_dummy_pixels;
			*pFeatureReturnPara16=SID130B_PV_PERIOD_LINE_NUMS;//+SID130B_PV_dummy_lines;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		//	*pFeatureReturnPara32 = SID130B_sensor_pclk/10;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
//			u2Temp = SID130B_read_shutter(); 
//			printk("Shutter:%d\n", u2Temp); 			
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			SID130B_NightMode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:
//			u2Temp = SID130B_read_gain(); 
//			printk("Gain:%d\n", u2Temp); 
//			printk("y_val:%d\n", SID130B_read_cmos_sensor(0x301B));
			break; 
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			//SID130B_isp_master_clock=*pFeatureData32;
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			SID130B_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = SID130B_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &SID130BSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:

		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_COUNT:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
		break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_YUV_CMD:
//		       printk("SID130B YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			SID130BYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		       SID130BYUVSetVideoMode(*pFeatureData16);
		       break; 
	   case SENSOR_FEATURE_CHECK_SENSOR_ID:
		   SID130BGetSensorID(pFeatureReturnPara32); 
		   break; 
		default:
			break;			
	}
	return ERROR_NONE;
}	/* SID130BFeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncSID130B=
{
	SID130BOpen,
	SID130BGetInfo,
	SID130BGetResolution,
	SID130BFeatureControl,
	SID130BControl,
	SID130BClose
};

UINT32 SID130B_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncSID130B;

	return ERROR_NONE;
}	/* SensorInit() */


