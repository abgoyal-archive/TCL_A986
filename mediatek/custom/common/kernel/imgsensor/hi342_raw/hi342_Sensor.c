

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi342_Sensor.h"
#include "hi342_Camera_Sensor_para.h"
#include "hi342_CameraCustomized.h"

#define HI342_DEBUG

#ifdef HI342_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif


typedef struct
{
  MSDK_SENSOR_CONFIG_STRUCT CfgData;
  sensor_data_struct Eng; /* engineer mode */
  MSDK_SENSOR_ENG_INFO_STRUCT EngInfo;
  kal_bool    PvMode;
  kal_uint32  Shutter;
  kal_uint16  Gain;  
  kal_uint32  DummyPixels;
  kal_uint32  DummyLines;
  kal_uint32  PvOpClk;
  kal_uint32  CapOpClk;
  kal_uint16   Fps;

  /*sensor register backup */
  kal_uint8   VDOCTL2;
  
} HI342Status;

static HI342Status HI342Sensor =
{
  .Eng =
  {
    .reg = HI342_CAMERA_SENSOR_REG_DEFAULT_VALUE,
    .cct = HI342_CAMERA_SENSOR_CCT_DEFAULT_VALUE,
  },
  .EngInfo =
  {
    .SensorId = 138,
    .SensorType = CMOS_SENSOR,
    .SensorOutputDataFormat = HI342_COLOR_FORMAT,
  },
  .PvMode = KAL_TRUE,
  .Shutter = 51,  
  .Gain = 0x80,
  .DummyPixels = 352,
  .DummyLines = 283,
  .PvOpClk = 455,
  .CapOpClk = 455,
  .VDOCTL2 = 0x90,
  .Fps = HI342_FPS(5),
};

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

#define Sleep(ms) mdelay(ms)

kal_uint16 HI342WriteCmosSensor(kal_uint32 Addr, kal_uint32 Para)
{
  char pSendCmd[2] = {(char)(Addr & 0xFF) ,(char)(Para & 0xFF)};
  
  //SENSORDB("[HI342]HI342WriteCmosSensor,Addr:%x;Para:%x \n",Addr,Para);
  iWriteRegI2C(pSendCmd , 2,HI342_WRITE_ID);
}

kal_uint16 HI342ReadCmosSensor(kal_uint32 Addr)
{
  char pGetByte=0;
  char pSendCmd = (char)(Addr & 0xFF);
  
  iReadRegI2C(&pSendCmd , 1, &pGetByte,1,HI342_WRITE_ID);
  return pGetByte;
}

void HI342SetPage(kal_uint8 Page)
{
  HI342WriteCmosSensor(0x03, Page);
}

static void HI342WriteShutter(kal_uint32 Shutter)
{ 
  HI342SetPage(0x20);
  HI342WriteCmosSensor(0x83, (Shutter>>16)&(0xff)); 
  HI342WriteCmosSensor(0x84, (Shutter>>8)&(0xff)); 
  HI342WriteCmosSensor(0x85, (Shutter>>0)&(0xff)); 
}   /*  HI342WriteShutter    */


static void HI342WriteDummy(const kal_uint16 Pixels, const kal_uint16 Lines)
{
  SENSORDB("[HI342]HI342WriteDummy Pixels: %d; Lines: %d\n",Pixels, Lines);
  HI342SetPage(0x00); 
  HI342WriteCmosSensor(0x40, (Pixels>>8)&0xff);
  HI342WriteCmosSensor(0x41, (Pixels>>0)&0xff);
  HI342WriteCmosSensor(0x42, (Lines>>8)&0xff);
  HI342WriteCmosSensor(0x43, (Lines>>0)&0xff); 
}   /*  HI342WriteDummy    */

void HI342SetShutter(kal_uint16 Shutter)
{
  kal_uint32 ExposureTime; /* The unit is 8x Opclk */
  kal_uint32 LineLength = HI342_PV_PERIOD_PIXEL_NUMS + HI342Sensor.DummyPixels;
  
  //SENSORDB("HI342WriteShutter:%x \n",Shutter);
  if (!Shutter) Shutter = 1; /* avoid 0 */
  
  ExposureTime = LineLength / 8 * Shutter;
  //SENSORDB("Exposure time:%x \n",ExposureTime);

  HI342Sensor.Shutter = ExposureTime;
  HI342WriteShutter(ExposureTime);

}   /*  Set_HI342_Shutter */

kal_uint16 HI342SetGain(kal_uint16 Gain)

{
  kal_uint8 Reg;
  
  //SENSORDB("HI342SetGain:%x;\n",Gain);
  /*
    AG is common gain for R, G and B channel and is used for AE operation.
    AG = 0.5 + B[7:0]/32. 0x00(0.5x) ~ 0xFF(8.5x)
  */  

  if (Gain < BASEGAIN * HI342_MIN_ANALOG_GAIN / HI342_GAIN_UNIT)
  Gain = BASEGAIN * HI342_MIN_ANALOG_GAIN / HI342_GAIN_UNIT;

  if (Gain > BASEGAIN * HI342_MAX_ANALOG_GAIN / HI342_GAIN_UNIT)
  Gain = BASEGAIN * HI342_MAX_ANALOG_GAIN / HI342_GAIN_UNIT;

  Reg = 32 * Gain / BASEGAIN - 16;

  HI342SetPage(0x20);
  HI342WriteCmosSensor(0xb0, Reg); 

  return Gain;
}


/* write camera_para to sensor register */
static void HI342CameraParaToSensor(void)
{
  kal_uint32 i;
  for (i = 0; 0xFFFFFFFF != HI342Sensor.Eng.reg[i].Addr; i++)
  {
    HI342WriteCmosSensor(HI342Sensor.Eng.reg[i].Addr, HI342Sensor.Eng.reg[i].Para);
  }
  for (i = HI342_FACTORY_START_ADDR; 0xFFFFFFFF != HI342Sensor.Eng.reg[i].Addr; i++)
  {
    HI342WriteCmosSensor(HI342Sensor.Eng.reg[i].Addr, HI342Sensor.Eng.reg[i].Para);
  }
  HI342SetGain(HI342Sensor.Gain); /* update Gain */
}

/* update camera_para from sensor register */
static void HI342SensorToCameraPara(void)
{
  kal_uint32 i;
  for (i = 0; 0xFFFFFFFF != HI342Sensor.Eng.reg[i].Addr; i++)
  {
    HI342Sensor.Eng.reg[i].Para = HI342ReadCmosSensor(HI342Sensor.Eng.reg[i].Addr);
  }
  for (i = HI342_FACTORY_START_ADDR; 0xFFFFFFFF != HI342Sensor.Eng.reg[i].Addr; i++)
  {
    HI342Sensor.Eng.reg[i].Para = HI342ReadCmosSensor(HI342Sensor.Eng.reg[i].Addr);
  }
}

/* ------------------------ Engineer mode ------------------------ */
inline static void HI342GetSensorGroupCount(kal_int32 *sensor_count_ptr)
{
  *sensor_count_ptr = HI342_GROUP_TOTAL_NUMS;
}

inline static void HI342GetSensorGroupInfo(MSDK_SENSOR_GROUP_INFO_STRUCT *Para)
{
  switch (Para->GroupIdx)
  {
  case HI342_PRE_GAIN:
    sprintf(Para->GroupNamePtr, "CCT");
    Para->ItemCount = 5;
    break;
  case HI342_CMMCLK_CURRENT:
    sprintf(Para->GroupNamePtr, "CMMCLK Current");
    Para->ItemCount = 1;
    break;
  case HI342_FRAME_RATE_LIMITATION:
    sprintf(Para->GroupNamePtr, "Frame Rate Limitation");
    Para->ItemCount = 2;
    break;
  case HI342_REGISTER_EDITOR:
    sprintf(Para->GroupNamePtr, "Register Editor");
    Para->ItemCount = 2;
    break;
  default:
    ASSERT(0);
  }
}

inline static void HI342GetSensorItemInfo(MSDK_SENSOR_ITEM_INFO_STRUCT *Para)
{

  const static kal_char *cct_item_name[] = {"SENSOR_BASEGAIN", "Pregain-R", "Pregain-Gr", "Pregain-Gb", "Pregain-B"};
  const static kal_char *editer_item_name[] = {"REG Addr", "REG value"};
  
  switch (Para->GroupIdx)
  {
  case HI342_PRE_GAIN:
    switch (Para->ItemIdx)
    {
    case HI342_SENSOR_BASEGAIN:
    case HI342_PRE_GAIN_R_INDEX:
    case HI342_PRE_GAIN_Gr_INDEX:
    case HI342_PRE_GAIN_Gb_INDEX:
    case HI342_PRE_GAIN_B_INDEX:
      break;
    default:
      ASSERT(0);
    }
    sprintf(Para->ItemNamePtr, cct_item_name[Para->ItemIdx - HI342_SENSOR_BASEGAIN]);
    Para->ItemValue = HI342Sensor.Eng.cct[Para->ItemIdx].Para * 1000 / BASEGAIN;
    Para->IsTrueFalse = Para->IsReadOnly = Para->IsNeedRestart = KAL_FALSE;
    Para->Min = HI342_MIN_ANALOG_GAIN / HI342_GAIN_UNIT * 1000;
    Para->Max = HI342_MAX_ANALOG_GAIN / HI342_GAIN_UNIT * 1000;
    break;
  case HI342_CMMCLK_CURRENT:
    switch (Para->ItemIdx)
    {
    case 0:
      sprintf(Para->ItemNamePtr, "Drv Cur[2,4,6,8]mA");
      switch (HI342Sensor.Eng.reg[HI342_CMMCLK_CURRENT_INDEX].Para)
      {
      case ISP_DRIVING_2MA:
        Para->ItemValue = 2;
        break;
      case ISP_DRIVING_4MA:
        Para->ItemValue = 4;
        break;
      case ISP_DRIVING_6MA:
        Para->ItemValue = 6;
        break;
      case ISP_DRIVING_8MA:
        Para->ItemValue = 8;
        break;
      default:
        ASSERT(0);
      }
      Para->IsTrueFalse = Para->IsReadOnly = KAL_FALSE;
      Para->IsNeedRestart = KAL_TRUE;
      Para->Min = 2;
      Para->Max = 8;
      break;
    default:
      ASSERT(0);
    }
    break;
  case HI342_FRAME_RATE_LIMITATION:
    switch (Para->ItemIdx)
    {
    case 0:
      sprintf(Para->ItemNamePtr, "Max Exposure Lines");
      Para->ItemValue = 5998;
      break;
    case 1:
      sprintf(Para->ItemNamePtr, "Min Frame Rate");
      Para->ItemValue = 5;
      break;
    default:
      ASSERT(0);
    }
    Para->IsTrueFalse = Para->IsNeedRestart = KAL_FALSE;
    Para->IsReadOnly = KAL_TRUE;
    Para->Min = Para->Max = 0;
    break;
  case HI342_REGISTER_EDITOR:
    switch (Para->ItemIdx)
    {
    case 0:
    case 1:
      sprintf(Para->ItemNamePtr, editer_item_name[Para->ItemIdx]);
      Para->ItemValue = 0;
      Para->IsTrueFalse = Para->IsReadOnly = Para->IsNeedRestart = KAL_FALSE;
      Para->Min = 0;
      Para->Max = (Para->ItemIdx == 0 ? 0xFFFF : 0xFF);
      break;
    default:
      ASSERT(0);
    }
    break;
  default:
    ASSERT(0);
  }
}

inline static kal_bool HI342SetSensorItemInfo(MSDK_SENSOR_ITEM_INFO_STRUCT *Para)
{
  kal_uint16 TempPara;
  switch (Para->GroupIdx)
  {
  case HI342_PRE_GAIN:
    switch (Para->ItemIdx)
    {
    case HI342_SENSOR_BASEGAIN:
    case HI342_PRE_GAIN_R_INDEX:
    case HI342_PRE_GAIN_Gr_INDEX:
    case HI342_PRE_GAIN_Gb_INDEX:
    case HI342_PRE_GAIN_B_INDEX:
      HI342Sensor.Eng.cct[Para->ItemIdx].Para = Para->ItemValue * BASEGAIN / 1000;
      HI342SetGain(HI342Sensor.Gain); /* update Gain */
      break;
    default:
      ASSERT(0);
    }
    break;
  case HI342_CMMCLK_CURRENT:
    switch (Para->ItemIdx)
    {
    case 0:
      switch (Para->ItemValue)
      {
      case 2:
        TempPara = ISP_DRIVING_2MA;
        break;
      case 3:
      case 4:
        TempPara = ISP_DRIVING_4MA;
        break;
      case 5:
      case 6:
        TempPara = ISP_DRIVING_6MA;
        break;
      default:
        TempPara = ISP_DRIVING_8MA;
        break;
      }
      HI342Sensor.Eng.reg[HI342_CMMCLK_CURRENT_INDEX].Para = TempPara;
      break;
    default:
      ASSERT(0);
    }
    break;
  case HI342_FRAME_RATE_LIMITATION:
    ASSERT(0);
    break;
  case HI342_REGISTER_EDITOR:
    switch (Para->ItemIdx)
    {
      static kal_uint32 fac_sensor_reg;
    case 0:
      if (Para->ItemValue < 0 || Para->ItemValue > 0xFFFF) return KAL_FALSE;
      fac_sensor_reg = Para->ItemValue;
      break;
    case 1:
      if (Para->ItemValue < 0 || Para->ItemValue > 0xFF) return KAL_FALSE;
      HI342WriteCmosSensor(fac_sensor_reg, Para->ItemValue);
      break;
    default:
      ASSERT(0);
    }
    break;
  default:
    ASSERT(0);
  }
  return KAL_TRUE;
}
void HI342InitSetting(void)
{
  HI342SetPage(0x00);
  HI342WriteCmosSensor(0x01, 0xf1); //sleep on 
  HI342WriteCmosSensor(0x08, 0x0f); //Hi-Z on  
  HI342WriteCmosSensor(0x01, 0xf0); //sleep off

  HI342SetPage(0x00); //Dummy 750us
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);

  HI342WriteCmosSensor(0x0e, 0x03);
  HI342WriteCmosSensor(0x0e, 0xe3); //PLL 2x

  HI342SetPage(0x00); //Dummy 750us
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  
  HI342WriteCmosSensor(0x0e, 0x03); //PLL off
  HI342WriteCmosSensor(0x01, 0xf1); //sleep on
  HI342WriteCmosSensor(0x08, 0x00); //Hi-Z off

  HI342WriteCmosSensor(0x01, 0xf3); //SW Reset
  HI342WriteCmosSensor(0x01, 0xf1); //sleep on

  HI342SetPage(0x00);
  HI342WriteCmosSensor(0x01, 0xf1); //sleep on
  HI342WriteCmosSensor(0x01, 0xf3); //SW Reset
  HI342WriteCmosSensor(0x01, 0xf1); //sleep on

  HI342SetPage(0x20); //page 20
  HI342WriteCmosSensor(0x10, 0x1c); //ae off

  HI342SetPage(0x22); //page 22
  HI342WriteCmosSensor(0x10, 0x69); //awb off 
  
  HI342SetPage(0x00);
  HI342WriteCmosSensor(0x0e, 0x06);
  HI342WriteCmosSensor(0x0e, 0xe6); //PLL 2x

  HI342SetPage(0x10);
  HI342WriteCmosSensor(0xe0, 0x00);
  HI342WriteCmosSensor(0xe1, 0x00);
  HI342WriteCmosSensor(0xe2, 0x00);
  HI342WriteCmosSensor(0xe3, 0x00);
  HI342WriteCmosSensor(0xe4, 0x00);
  HI342WriteCmosSensor(0xe5, 0x00);
  HI342WriteCmosSensor(0xe6, 0x30);
  HI342WriteCmosSensor(0xe7, 0x00);
  HI342WriteCmosSensor(0xe8, 0x00);

  HI342WriteCmosSensor(0xea, 0x00);
  HI342WriteCmosSensor(0xeb, 0x00);
  HI342WriteCmosSensor(0xec, 0x00);
  HI342WriteCmosSensor(0xed, 0x00);
  HI342WriteCmosSensor(0xf0, 0x40);
  HI342WriteCmosSensor(0xf2, 0x00);
  HI342WriteCmosSensor(0xf3, 0x00);
  HI342WriteCmosSensor(0xf8, 0xa8);
  HI342WriteCmosSensor(0xf9, 0xb0);

  HI342SetPage(0x20);
  HI342WriteCmosSensor(0x1a, 0x24);
  HI342WriteCmosSensor(0x1b, 0x00);

  HI342SetPage(0x00);
  HI342WriteCmosSensor(0x10, 0x13);
  HI342WriteCmosSensor(0x11, 0x90);
  HI342WriteCmosSensor(0x12, 0x04); 
  HI342WriteCmosSensor(0x13, 0x00);

  HI342WriteCmosSensor(0x20, 0x00); // WINROWH
  HI342WriteCmosSensor(0x21, 0x04); // WINROWL
  HI342WriteCmosSensor(0x22, 0x00); // WINCOLH
  HI342WriteCmosSensor(0x23, 0x07); // WINCOLL
  HI342WriteCmosSensor(0x24, 0x06); 
  HI342WriteCmosSensor(0x25, 0x00); 
  HI342WriteCmosSensor(0x26, 0x08); 
  HI342WriteCmosSensor(0x27, 0x00); 

  HI342WriteCmosSensor(0x32, 0x06);
  HI342WriteCmosSensor(0x33, 0x30);
  HI342WriteCmosSensor(0x34, 0x06);
  HI342WriteCmosSensor(0x35, 0x30);

  HI342WriteCmosSensor(0x40, 0x00); //HBALNK 248
  HI342WriteCmosSensor(0x41, 0xf8);

  HI342WriteCmosSensor(0x42, 0x00); //VSYNC 04
  HI342WriteCmosSensor(0x43, 0x04);

  HI342WriteCmosSensor(0x80, 0x00); //BLC OFF 08 -> 00

  HI342SetPage(0x02);
  HI342WriteCmosSensor(0x10, 0x00);
  HI342WriteCmosSensor(0x12, 0x7F);
  HI342WriteCmosSensor(0x13, 0x00);
  HI342WriteCmosSensor(0x18, 0x1C);
  HI342WriteCmosSensor(0x19, 0x00);
  HI342WriteCmosSensor(0x1A, 0x39);
  HI342WriteCmosSensor(0x1B, 0x00);
  HI342WriteCmosSensor(0x16, 0x00);
  HI342WriteCmosSensor(0x17, 0x00);

  HI342WriteCmosSensor(0x1F, 0x10);
  HI342WriteCmosSensor(0x20, 0x77);
  HI342WriteCmosSensor(0x21, 0xed);
  HI342WriteCmosSensor(0x22, 0xA7);
  HI342WriteCmosSensor(0x23, 0x30);
  HI342WriteCmosSensor(0x25, 0x10);
  HI342WriteCmosSensor(0x27, 0x34);
  HI342WriteCmosSensor(0x2B, 0x84);
  HI342WriteCmosSensor(0x2E, 0x11);
  HI342WriteCmosSensor(0x2F, 0xA1);
  HI342WriteCmosSensor(0x30, 0x00);
  HI342WriteCmosSensor(0x31, 0x99);
  HI342WriteCmosSensor(0x32, 0x00);
  HI342WriteCmosSensor(0x33, 0x00);
  HI342WriteCmosSensor(0x34, 0x22);
  HI342WriteCmosSensor(0x35, 0x01);
  HI342WriteCmosSensor(0x36, 0x01);
  HI342WriteCmosSensor(0x37, 0x01);
  HI342WriteCmosSensor(0x3D, 0x03);
  HI342WriteCmosSensor(0x3e, 0x0d);
  HI342WriteCmosSensor(0x49, 0xD1);
  HI342WriteCmosSensor(0x50, 0x28);
  HI342WriteCmosSensor(0x52, 0x03);
  HI342WriteCmosSensor(0x53, 0x81);
  HI342WriteCmosSensor(0x54, 0x3C);
  HI342WriteCmosSensor(0x55, 0x1C);
  HI342WriteCmosSensor(0x56, 0x11);
  HI342WriteCmosSensor(0x5d, 0xa2);
  HI342WriteCmosSensor(0x5E, 0x55);
  HI342WriteCmosSensor(0x60, 0x8d);
  HI342WriteCmosSensor(0x61, 0x9b);
  HI342WriteCmosSensor(0x62, 0x8e);
  HI342WriteCmosSensor(0x63, 0x99);
  HI342WriteCmosSensor(0x64, 0x8e);
  HI342WriteCmosSensor(0x65, 0x99);
  HI342WriteCmosSensor(0x67, 0x0c);
  HI342WriteCmosSensor(0x68, 0x0c);
  HI342WriteCmosSensor(0x69, 0x0c);

  HI342WriteCmosSensor(0x72, 0x8e);
  HI342WriteCmosSensor(0x73, 0x98);
  HI342WriteCmosSensor(0x74, 0x8e);
  HI342WriteCmosSensor(0x75, 0x98);

  HI342WriteCmosSensor(0x76, 0xa6);
  HI342WriteCmosSensor(0x77, 0xb6);
  HI342WriteCmosSensor(0x7C, 0xa0);
  HI342WriteCmosSensor(0x7d, 0xd0);

  HI342WriteCmosSensor(0x80, 0x01);
  HI342WriteCmosSensor(0x81, 0x82);
  HI342WriteCmosSensor(0x82, 0x1e);
  HI342WriteCmosSensor(0x83, 0x30);
  HI342WriteCmosSensor(0x84, 0x80);
  HI342WriteCmosSensor(0x85, 0x83);
  HI342WriteCmosSensor(0x86, 0x80);
  HI342WriteCmosSensor(0x87, 0x83);

  HI342WriteCmosSensor(0x92, 0x4e);
  HI342WriteCmosSensor(0x93, 0x60);
  HI342WriteCmosSensor(0x94, 0x80);
  HI342WriteCmosSensor(0x95, 0x83);
  HI342WriteCmosSensor(0x96, 0x80);
  HI342WriteCmosSensor(0x97, 0x83);

  HI342WriteCmosSensor(0xa0, 0x02);
  HI342WriteCmosSensor(0xa1, 0x7d);
  HI342WriteCmosSensor(0xa2, 0x02);
  HI342WriteCmosSensor(0xa3, 0x7d);
  HI342WriteCmosSensor(0xa5, 0x02);
  HI342WriteCmosSensor(0xa4, 0x7d);
  HI342WriteCmosSensor(0xa6, 0x7d);
  HI342WriteCmosSensor(0xa7, 0x02);
  HI342WriteCmosSensor(0xa8, 0x8d);
  HI342WriteCmosSensor(0xa9, 0x93);
  HI342WriteCmosSensor(0xaa, 0x8d);
  HI342WriteCmosSensor(0xab, 0x93);
  HI342WriteCmosSensor(0xac, 0x1b);
  HI342WriteCmosSensor(0xad, 0x22);
  HI342WriteCmosSensor(0xae, 0x1b);
  HI342WriteCmosSensor(0xaf, 0x22);

  HI342WriteCmosSensor(0xb0, 0xa0);
  HI342WriteCmosSensor(0xb1, 0xaa);
  HI342WriteCmosSensor(0xb4, 0xa1);
  HI342WriteCmosSensor(0xb5, 0xa8);
  HI342WriteCmosSensor(0xb8, 0xa2);
  HI342WriteCmosSensor(0xb9, 0xa7);
  HI342WriteCmosSensor(0xbc, 0xa2);
  HI342WriteCmosSensor(0xbd, 0xa6);
  HI342WriteCmosSensor(0xb2, 0xa0);
  HI342WriteCmosSensor(0xb3, 0xaa);
  HI342WriteCmosSensor(0xb6, 0xa1);
  HI342WriteCmosSensor(0xb7, 0xa8);
  HI342WriteCmosSensor(0xba, 0xa2);
  HI342WriteCmosSensor(0xbb, 0xa7);
  HI342WriteCmosSensor(0xbe, 0xa2);
  HI342WriteCmosSensor(0xbf, 0xa6);

  HI342WriteCmosSensor(0xc4, 0x32);
  HI342WriteCmosSensor(0xc5, 0x49);
  HI342WriteCmosSensor(0xc6, 0x63);
  HI342WriteCmosSensor(0xc7, 0x7a);
  HI342WriteCmosSensor(0xc8, 0x33);
  HI342WriteCmosSensor(0xc9, 0x48);
  HI342WriteCmosSensor(0xca, 0x33);
  HI342WriteCmosSensor(0xcb, 0x48);
  HI342WriteCmosSensor(0xcc, 0x64);
  HI342WriteCmosSensor(0xcd, 0x79);
  HI342WriteCmosSensor(0xce, 0x64);
  HI342WriteCmosSensor(0xcf, 0x79);
  HI342WriteCmosSensor(0xea, 0x82);

  HI342WriteCmosSensor(0xeb, 0x02);
  HI342WriteCmosSensor(0xef, 0x02);

  HI342WriteCmosSensor(0x1c, 0x3c);
  HI342WriteCmosSensor(0x1d, 0x01);
  HI342WriteCmosSensor(0x1e, 0x30);

  HI342WriteCmosSensor(0xd0, 0x0a);
  HI342WriteCmosSensor(0xd1, 0x09);
  HI342WriteCmosSensor(0xd2, 0x20);
  HI342WriteCmosSensor(0xd3, 0x00);

  HI342WriteCmosSensor(0xd4, 0x0a);
  HI342WriteCmosSensor(0xd5, 0x0a);
  HI342WriteCmosSensor(0xd6, 0x88);
  HI342WriteCmosSensor(0xd7, 0x80);

  HI342WriteCmosSensor(0xe0, 0xe1);
  HI342WriteCmosSensor(0xe1, 0xe1);
  HI342WriteCmosSensor(0xe2, 0xe1);
  HI342WriteCmosSensor(0xe3, 0xe1);
  HI342WriteCmosSensor(0xe4, 0xe1);
  HI342WriteCmosSensor(0xe5, 0x01);

  HI342SetPage(0x10);
  HI342WriteCmosSensor(0x10, 0x91);
  HI342WriteCmosSensor(0x11, 0x03);
  HI342WriteCmosSensor(0x12, 0xc0);
  HI342WriteCmosSensor(0x13, 0x0a);

  HI342WriteCmosSensor(0x48, 0x84);

  HI342WriteCmosSensor(0x60, 0x00); //Saturation Off

  HI342SetPage(0x11);
  HI342WriteCmosSensor(0x10, 0x0f);// D-LPF Off

  HI342SetPage(0x12);
  HI342WriteCmosSensor(0x12, 0x3f);
  HI342WriteCmosSensor(0x20, 0x32);

  HI342WriteCmosSensor(0x25, 0x07);
  HI342WriteCmosSensor(0x26, 0x07);
  HI342WriteCmosSensor(0x27, 0x07);

  HI342WriteCmosSensor(0x3e, 0x03);

  HI342WriteCmosSensor(0x51, 0x44);
  HI342WriteCmosSensor(0x52, 0x44);
  HI342WriteCmosSensor(0x54, 0x00);
  HI342WriteCmosSensor(0x55, 0x03);

  HI342WriteCmosSensor(0x68, 0x78);
  HI342WriteCmosSensor(0x69, 0x78);
  HI342WriteCmosSensor(0x6c, 0x40);
  HI342WriteCmosSensor(0x6f, 0x80);

  HI342WriteCmosSensor(0x70, 0x00);// Lens Deblur off
  HI342SetPage(0x13);
  HI342WriteCmosSensor(0x20, 0x00);// Yc2d LPF Off

  HI342SetPage(0x14);
  HI342WriteCmosSensor(0x10, 0x00);

  HI342WriteCmosSensor(0x80, 0x00);

  HI342SetPage(0x15);
  HI342WriteCmosSensor(0x10, 0x00);

  HI342SetPage(0x17);
  HI342WriteCmosSensor(0x10, 0x00);// Gamma OFF

  HI342SetPage(0x16);
  HI342WriteCmosSensor(0x10, 0x00);// CMC Off

  HI342SetPage(0x20);
  HI342WriteCmosSensor(0x10, 0x00);//AE Off

  HI342SetPage(0x22);
  HI342WriteCmosSensor(0x10, 0x00);//AWB Off

  HI342SetPage(0x00);
  HI342WriteCmosSensor(0x0e, 0x06);
  HI342WriteCmosSensor(0x0e, 0xe6); //PLL 2x

  HI342SetPage(0x00); //Dummy 750us
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);
  HI342SetPage(0x00);

  HI342WriteCmosSensor(0x01, 0xf0); //sleep off
  //HI342WriteCmosSensor(0xff, 0x10); //Delay 100ms
}


static void HI342SetMirror(kal_uint16 ImageMirror)
{
  HI342Sensor.VDOCTL2 &=0xfc; 
  switch (ImageMirror)
  {
    case IMAGE_H_MIRROR:
      HI342Sensor.VDOCTL2 |= 0x01;
      break;
    case IMAGE_V_MIRROR:
      HI342Sensor.VDOCTL2 |= 0x02; 
      break;
    case IMAGE_HV_MIRROR:
      HI342Sensor.VDOCTL2 |= 0x03;
      break;
    case IMAGE_NORMAL:
    default:
      HI342Sensor.VDOCTL2 |= 0x00; 
  }
  HI342SetPage(0x00);
  HI342WriteCmosSensor(0x11,HI342Sensor.VDOCTL2);  
}

static void HI342FixFrameRate(kal_bool Enable)
{
  kal_uint32 MaxShutter;
  kal_uint32 FixShutter;
  kal_uint16 LineLength = HI342Sensor.DummyPixels + HI342_PV_PERIOD_PIXEL_NUMS;
  
  HI342Sensor.VDOCTL2 &=0xfb; 
  HI342Sensor.VDOCTL2 |= Enable ? 0x4 : 0x0;
  
  SENSORDB("[HI342]HI342FixFrameRate Enable:%d;\n",Enable);

  HI342SetPage(0x00);
  //HI342WriteCmosSensor(0x01, 0xf1); // Sleep ON
  
  HI342WriteCmosSensor(0x11,HI342Sensor.VDOCTL2);  
  
  HI342SetPage(0x20);
  MaxShutter = HI342Sensor.PvOpClk * HI342_CLOCK_UNIT / 8 / HI342Sensor.Fps * HI342_FPS(1);
  SENSORDB("[HI342]MaxShutter:%d;\n",MaxShutter);
  if (KAL_TRUE == Enable)
  {
    FixShutter = (HI342Sensor.PvOpClk * HI342_CLOCK_UNIT / HI342Sensor.Fps * HI342_FPS(1) / LineLength - HI342Sensor.DummyLines) * LineLength / 8;
    SENSORDB("[HI342]FixShutter:%d;\n",FixShutter);
    HI342WriteCmosSensor(0x91, (FixShutter>>16)&(0xff)); 
    HI342WriteCmosSensor(0x92, (FixShutter>>8)&(0xff)); 
    HI342WriteCmosSensor(0x93, (FixShutter>>0)&(0xff)); 
  }
  HI342WriteCmosSensor(0x88, (MaxShutter>>16)&(0xff)); 
  HI342WriteCmosSensor(0x89, (MaxShutter>>8)&(0xff)); 
  HI342WriteCmosSensor(0x8a, (MaxShutter>>0)&(0xff)); 

  //HI342SetPage(0x00);  
  //HI342WriteCmosSensor(0x01, 0xf0); // Sleep OFF  

}
void HI342NightMode(kal_bool Enable)
{
  SENSORDB("[HI342]HI342NightMode Enable:%d;\n",Enable);

} /* HI342NightMode */

UINT32 HI342Open(void)
{
  kal_uint16 SensorId = 0;
  //1 software reset sensor and wait (to sensor)
  HI342SetPage(0x00);
  HI342WriteCmosSensor(0x01,0xf1);
  HI342WriteCmosSensor(0x01,0xf3);
  HI342WriteCmosSensor(0x01,0xf1);

  SensorId = HI342ReadCmosSensor(0x04);
  Sleep(3);
  SENSORDB("[HI342]HI342Open: Sensor ID %x\n",SensorId);
  
  if(SensorId != HI342_SENSOR_ID)
  {
    return ERROR_SENSOR_CONNECT_FAIL;
  }
  HI342InitSetting();
  return ERROR_NONE;

}
/* HI342Open() */

UINT32 HI342GetSensorID(UINT32 *sensorID) 
{
	//1 software reset sensor and wait (to sensor)
	HI342SetPage(0x00);
	HI342WriteCmosSensor(0x01,0xf1);
	HI342WriteCmosSensor(0x01,0xf3);
	HI342WriteCmosSensor(0x01,0xf1);
	
	*sensorID = HI342ReadCmosSensor(0x04);
	Sleep(3);
	SENSORDB("[HI342]HI342GetSensorID: Sensor ID %x\n",*sensorID);
	
	if(*sensorID != HI342_SENSOR_ID)
	{
	  return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

UINT32 HI342Close(void)
{
  return ERROR_NONE;
} /* HI342Close() */

UINT32 HI342Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

  SENSORDB("\n\n\n\n\n\n");
  SENSORDB("[HI342]HI342Preview\n");

  HI342Sensor.PvMode = KAL_TRUE;
  
  HI342Sensor.DummyPixels = 352;
  /* DummyLines = Pclk / MaxFps / LineLength - HI342_PV_PERIOD_LINE_NUMS */
  HI342Sensor.DummyLines = 283; 
  HI342Sensor.Fps = HI342_FPS(5);

  HI342SetMirror(IMAGE_NORMAL);
  HI342FixFrameRate(KAL_FALSE);  
  
  HI342SetPage(0x00); 
  HI342WriteCmosSensor(0x01, 0xf1); // Sleep ON
  HI342WriteCmosSensor(0x10, 0x13); 
  
  HI342WriteDummy(HI342Sensor.DummyPixels,HI342Sensor.DummyLines);
  //HI342WriteShutter(HI342Sensor.Shutter);
  
  HI342SetPage(0x00);  
  HI342WriteCmosSensor(0x01, 0xf0); // Sleep OFF  

  return ERROR_NONE;
}/* HI342Preview() */

UINT32 HI342Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
  kal_uint32  EXP100, EXP120, EXPMIN, EXPUNIT, CapShutter; 
  kal_uint8 ClockDivider;
  kal_uint16 MaxFps;
  kal_uint32 Shutter;
  
  SENSORDB("\n\n\n\n\n\n");
  SENSORDB("[HI342]HI342Capture!!!!!!!!!!!!!\n");

  HI342Sensor.PvMode = KAL_FALSE;
  HI342Sensor.DummyPixels = 512;   /* Max frame rate is 11 fps, when dummy line is 7*/

  MaxFps = HI342_FPS(10); 
  /* DummyLines = Pclk / MaxFps / LineLength - HI342_FULL_PERIOD_LINE_NUMS */
  HI342Sensor.DummyLines = HI342Sensor.CapOpClk * 100000 * HI342_FPS(1)/ MaxFps /(HI342Sensor.DummyPixels + HI342_FULL_PERIOD_PIXEL_NUMS); 
  HI342Sensor.DummyLines -= HI342_FULL_PERIOD_LINE_NUMS;
  SENSORDB("[HI342]Cap DummyLines: %d;\n",HI342Sensor.DummyLines);

  if (HI342Sensor.PvOpClk != HI342Sensor.CapOpClk)
  {
    Shutter = HI342Sensor.PvOpClk * HI342Sensor.Shutter / HI342Sensor.CapOpClk;
    HI342WriteShutter(Shutter);
    SENSORDB("[HI342]PV shutter: %x; Cap shutter: %x;n",HI342Sensor.Shutter, Shutter);
  }

  HI342SetPage(0x00);  
  HI342WriteCmosSensor(0x01, 0xf1); // Sleep ON
  // 1600*1200   
  HI342WriteCmosSensor(0x10,0x00);
  HI342WriteDummy(HI342Sensor.DummyPixels,HI342Sensor.DummyLines);
  HI342SetPage(0x00);  
  HI342WriteCmosSensor(0x01, 0xf0); // Sleep OFF  
  return ERROR_NONE;
} /* HI342Capture() */

UINT32 HI342GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
  pSensorResolution->SensorFullWidth = HI342_FULL_WIDTH;
  pSensorResolution->SensorFullHeight = HI342_FULL_HEIGHT;
  pSensorResolution->SensorPreviewWidth = HI342_PV_WIDTH;
  pSensorResolution->SensorPreviewHeight = HI342_PV_HEIGHT;
  return ERROR_NONE;
} /* HI342GetResolution() */

UINT32 HI342GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                    MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
  pSensorInfo->SensorPreviewResolutionX=HI342_PV_WIDTH;
  pSensorInfo->SensorPreviewResolutionY=HI342_PV_HEIGHT;
  pSensorInfo->SensorFullResolutionX=HI342_FULL_WIDTH;
  pSensorInfo->SensorFullResolutionY=HI342_FULL_HEIGHT;

  pSensorInfo->SensorCameraPreviewFrameRate=30;
  pSensorInfo->SensorVideoFrameRate=30;
  pSensorInfo->SensorStillCaptureFrameRate=10;
  pSensorInfo->SensorWebCamCaptureFrameRate=15;
  pSensorInfo->SensorResetActiveHigh=FALSE;
  pSensorInfo->SensorResetDelayCount=1;
  pSensorInfo->SensorOutputDataFormat=HI342_COLOR_FORMAT;
  pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
  pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
  pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
  pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
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
  pSensorInfo->CaptureDelayFrame = 3; 
  pSensorInfo->PreviewDelayFrame = 3; 
  pSensorInfo->VideoDelayFrame = 8; 
  pSensorInfo->SensorMasterClockSwitch = 0; 
  pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA; 

  pSensorInfo->AEShutDelayFrame = 1;        /* The frame of setting shutter default 0 for TG int */
  pSensorInfo->AESensorGainDelayFrame = 1;  /* The frame of setting sensor gain */
  pSensorInfo->AEISPGainDelayFrame = 1;    

  switch (ScenarioId)
  {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
    default:
      pSensorInfo->SensorClockFreq= 26 ;
      pSensorInfo->SensorClockDividCount=3;
      pSensorInfo->SensorClockRisingCount=0;
      pSensorInfo->SensorClockFallingCount=2;
      pSensorInfo->SensorPixelClockCount=3;
      pSensorInfo->SensorDataLatchCount=2;
      pSensorInfo->SensorGrabStartX = HI342_GRAB_START_X; 
      pSensorInfo->SensorGrabStartY = HI342_GRAB_START_Y;
      break;
  }
  return ERROR_NONE;
} /* HI342GetInfo() */


UINT32 HI342Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
  switch (ScenarioId)
  {
  case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
  case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
  case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
    HI342Preview(pImageWindow, pSensorConfigData);
    break;
  case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
  case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
    HI342Capture(pImageWindow, pSensorConfigData);
    break;
  default:
    break; 
  }
  return TRUE;
} /* HI342Control() */
UINT32 HI342SetVideoMode(UINT16 FrameRate)
{
  /* to fix VSYNC, to fix frame rate */
  SENSORDB("HI342SetVideoMode£¬u2FrameRate:%d\n",FrameRate);
  if (FrameRate == 30)
  {
    HI342Sensor.DummyPixels = 272;
    HI342Sensor.DummyLines = 4;
  }
  else if (FrameRate == 15)
  {
    HI342Sensor.DummyPixels = 272;
    HI342Sensor.DummyLines = 1153;
  }
  else
  {
    SENSORDB("Wrong frame rate setting %d\n", FrameRate);
    return KAL_FALSE;
  }
  HI342Sensor.Fps = FrameRate * HI342_FPS(1);
  /* modify DummyPixel must gen AE table again */
  HI342WriteDummy(HI342Sensor.DummyPixels, HI342Sensor.DummyLines); 
  HI342FixFrameRate(KAL_TRUE);  

  return TRUE;
}
UINT32 HI342FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                        UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
  UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
  UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
  UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
  UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
  UINT32 HI342SensorRegNumber;
  UINT32 i;
  PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
  MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
  MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
  MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
  MSDK_SENSOR_ENG_INFO_STRUCT *pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

  //SENSORDB("HI342FeatureControl£¬FeatureId:%d\n",FeatureId);
  switch (FeatureId)
  {
    case SENSOR_FEATURE_GET_RESOLUTION:
      *pFeatureReturnPara16++=HI342_FULL_WIDTH;
      *pFeatureReturnPara16=HI342_FULL_HEIGHT;
      *pFeatureParaLen=4;
      break;
    case SENSOR_FEATURE_GET_PERIOD: /* 2 */
      *pFeatureReturnPara16++=HI342Sensor.DummyPixels + HI342_PV_PERIOD_PIXEL_NUMS;
      *pFeatureReturnPara16=HI342Sensor.DummyLines + HI342_PV_PERIOD_LINE_NUMS;
      *pFeatureParaLen=4;
      break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:  /* 3 */
      *pFeatureReturnPara32 = HI342Sensor.PvOpClk * HI342_CLOCK_UNIT;
      *pFeatureParaLen=4;
      break;
    case SENSOR_FEATURE_SET_ESHUTTER: /* 4 */
      HI342SetShutter(*pFeatureData16);
      break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
      HI342NightMode((BOOL) *pFeatureData16);
      break;
    case SENSOR_FEATURE_SET_GAIN: /* 6 */
      HI342SetGain((UINT16) *pFeatureData16);
      break;
    case SENSOR_FEATURE_SET_FLASHLIGHT:
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
      break;
    case SENSOR_FEATURE_SET_REGISTER:
      HI342WriteCmosSensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
      break;
    case SENSOR_FEATURE_GET_REGISTER:
      pSensorRegData->RegData = HI342ReadCmosSensor(pSensorRegData->RegAddr);
      break;
    case SENSOR_FEATURE_SET_CCT_REGISTER:
      memcpy(&HI342Sensor.Eng.cct, pFeaturePara, sizeof(HI342Sensor.Eng.cct));
      break;
    case SENSOR_FEATURE_GET_CCT_REGISTER: /* 12 */
      if (*pFeatureParaLen >= sizeof(HI342Sensor.Eng.cct) + sizeof(kal_uint32))
      {
        *((kal_uint32 *)pFeaturePara++) = sizeof(HI342Sensor.Eng.cct);
        memcpy(pFeaturePara, &HI342Sensor.Eng.cct, sizeof(HI342Sensor.Eng.cct));
      }
      break;
    case SENSOR_FEATURE_SET_ENG_REGISTER:
      memcpy(&HI342Sensor.Eng.reg, pFeaturePara, sizeof(HI342Sensor.Eng.reg));
      break;
    case SENSOR_FEATURE_GET_ENG_REGISTER: /* 14 */
      if (*pFeatureParaLen >= sizeof(HI342Sensor.Eng.reg) + sizeof(kal_uint32))
      {
        *((kal_uint32 *)pFeaturePara++) = sizeof(HI342Sensor.Eng.reg);
        memcpy(pFeaturePara, &HI342Sensor.Eng.reg, sizeof(HI342Sensor.Eng.reg));
      }
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
      ((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->Version = NVRAM_CAMERA_SENSOR_FILE_VERSION;
      ((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorId = HI342_SENSOR_ID;
      memcpy(((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorEngReg, &HI342Sensor.Eng.reg, sizeof(HI342Sensor.Eng.reg));
      memcpy(((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorCCTReg, &HI342Sensor.Eng.cct, sizeof(HI342Sensor.Eng.cct));
      *pFeatureParaLen = sizeof(NVRAM_SENSOR_DATA_STRUCT);
      break;
    case SENSOR_FEATURE_GET_CONFIG_PARA:
      memcpy(pFeaturePara, &HI342Sensor.CfgData, sizeof(HI342Sensor.CfgData));
      *pFeatureParaLen = sizeof(HI342Sensor.CfgData);
      break;
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
      HI342CameraParaToSensor();
      break;
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
      HI342SensorToCameraPara();
      break;       
    case SENSOR_FEATURE_GET_GROUP_COUNT:
      HI342GetSensorGroupCount((kal_uint32 *)pFeaturePara);
      *pFeatureParaLen = 4;
      break; 
    case SENSOR_FEATURE_GET_GROUP_INFO:
      HI342GetSensorGroupInfo((MSDK_SENSOR_GROUP_INFO_STRUCT *)pFeaturePara);
      *pFeatureParaLen = sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
      break;
    case SENSOR_FEATURE_GET_ITEM_INFO:
      HI342GetSensorItemInfo((MSDK_SENSOR_ITEM_INFO_STRUCT *)pFeaturePara);
      *pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
      break;
    case SENSOR_FEATURE_SET_ITEM_INFO:
      HI342SetSensorItemInfo((MSDK_SENSOR_ITEM_INFO_STRUCT *)pFeaturePara);
      *pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
      break;
    case SENSOR_FEATURE_GET_ENG_INFO:
      memcpy(pFeaturePara, &HI342Sensor.EngInfo, sizeof(HI342Sensor.EngInfo));
      *pFeatureParaLen = sizeof(HI342Sensor.EngInfo);
      break;
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
      // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
      // if EEPROM does not exist in camera module.
      *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
      *pFeatureParaLen=4;
      break;
    case SENSOR_FEATURE_SET_VIDEO_MODE:
      HI342SetVideoMode(*pFeatureData16);
      break; 
  case SENSOR_FEATURE_CHECK_SENSOR_ID:
	  HI342GetSensorID(pFeatureReturnPara32); 
	  break; 
    default:
      break;
  }
  return ERROR_NONE;
} /* HI342FeatureControl() */



UINT32 HI342SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
  static SENSOR_FUNCTION_STRUCT SensorFuncHI342=
  {
    HI342Open,
    HI342GetInfo,
    HI342GetResolution,
    HI342FeatureControl,
    HI342Control,
    HI342Close
  };

  /* To Do : Check Sensor status here */
  if (pfFunc!=NULL)
    *pfFunc=&SensorFuncHI342;

  return ERROR_NONE;
} /* SensorInit() */
