/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H

#define HI342_FACTORY_START_ADDR 0
#define HI342_ENGINEER_START_ADDR 10
 
typedef enum HI342_group_enum
{
  HI342_PRE_GAIN = 0,
  HI342_CMMCLK_CURRENT,
  HI342_FRAME_RATE_LIMITATION,
  HI342_REGISTER_EDITOR,
  HI342_GROUP_TOTAL_NUMS
} HI342_FACTORY_GROUP_ENUM;

typedef enum HI342_register_index
{
  HI342_SENSOR_BASEGAIN = HI342_FACTORY_START_ADDR,
  HI342_PRE_GAIN_R_INDEX,
  HI342_PRE_GAIN_Gr_INDEX,
  HI342_PRE_GAIN_Gb_INDEX,
  HI342_PRE_GAIN_B_INDEX,
  HI342_FACTORY_END_ADDR
} HI342_FACTORY_REGISTER_INDEX;

typedef enum HI342_engineer_index
{
  HI342_CMMCLK_CURRENT_INDEX = HI342_ENGINEER_START_ADDR,
  HI342_ENGINEER_END
} HI342_FACTORY_ENGINEER_INDEX;

typedef struct _sensor_data_struct
{
  SENSOR_REG_STRUCT reg[HI342_ENGINEER_END];
  SENSOR_REG_STRUCT cct[HI342_FACTORY_END_ADDR];
} sensor_data_struct;


#define HI342_WRITE_ID        0x40

#define HI342_COLOR_FORMAT   SENSOR_OUTPUT_FORMAT_RAW_Gb



#define HI342_GRAB_START_X    (1)
#define HI342_GRAB_START_Y    (1)
#define HI342_PV_WIDTH        (1024 - 32)
#define HI342_PV_HEIGHT       (768 - 24)
#define HI342_FULL_WIDTH      (2048 - 64)
#define HI342_FULL_HEIGHT     (1536 - 48)

/* Sesnor Pixel/Line Numbers in One Period */  
#define HI342_PV_PERIOD_PIXEL_NUMS      (1048)    /* Default preview line length */
#define HI342_PV_PERIOD_LINE_NUMS       (800)     /* Default preview frame length */
#define HI342_FULL_PERIOD_PIXEL_NUMS    (2088)    /* Default full size line length */
#define HI342_FULL_PERIOD_LINE_NUMS     (1584)    /* Default full size frame length */

/* Sensor Exposure Line Limitation */
#define HI342_PV_EXPOSURE_LIMITATION        (800)
#define HI342_FULL_EXPOSURE_LIMITATION      (1584)

#define HI342_MIN_ANALOG_GAIN         5  /* 0.5x */
#define HI342_MAX_ANALOG_GAIN         85 /* 8.5x */
#define HI342_GAIN_UNIT               10

#define HI342_CLOCK_UNIT              100000
#define HI342_FRAME_RATE_UNIT         10
#define HI342_FPS(x)                  (HI342_FRAME_RATE_UNIT * (x))
#define HI342_MAX_FPS                 (HI342_FRAME_RATE_UNIT * 30)

UINT32 HI342Open(void);
UINT32 HI342GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 HI342GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 HI342Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 HI342FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 HI342Close(void);
#endif /* __SENSOR_H */
