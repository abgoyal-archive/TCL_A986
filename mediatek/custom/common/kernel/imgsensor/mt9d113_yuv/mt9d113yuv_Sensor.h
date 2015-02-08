
#ifndef __SENSOR_MT9D113_H
	#define __SENSOR_MT9D113_H


#define __MT9D113_DEBUG_TRACE__

#ifdef __MT9D113_DEBUG_TRACE__
	#define MT9D113_TRACE printk
#else
	#define MT9D113_TRACE(x,...)
#endif


#define MT9D113_REV2
#define MT9D113_REV3

/* SENSOR FULL/PV SIZE */
#define MT9D113_IMAGE_SENSOR_FULL_WIDTH_DRV   1600
#define MT9D113_IMAGE_SENSOR_FULL_HEIGHT_DRV  1200
#define MT9D113_IMAGE_SENSOR_PV_WIDTH_DRV     (MT9D113_IMAGE_SENSOR_FULL_WIDTH_DRV / 2)
#define MT9D113_IMAGE_SENSOR_PV_HEIGHT_DRV    (MT9D113_IMAGE_SENSOR_FULL_HEIGHT_DRV / 2)

/* SENSOR HORIZONTAL/VERTICAL ACTIVE REGION */
#define MT9D113_IMAGE_SENSOR_FULL_HACTIVE     MT9D113_IMAGE_SENSOR_FULL_WIDTH_DRV /* 1600 */
#define MT9D113_IMAGE_SENSOR_FULL_VACTIVE     MT9D113_IMAGE_SENSOR_FULL_HEIGHT_DRV /* 1200 */
#define MT9D113_IMAGE_SENSOR_PV_HACTIVE       MT9D113_IMAGE_SENSOR_PV_WIDTH_DRV /* 800 */
#define MT9D113_IMAGE_SENSOR_PV_VACTIVE       MT9D113_IMAGE_SENSOR_PV_HEIGHT_DRV /* 600 */

/* SENSOR HORIZONTAL/VERTICAL BLANKING IN ONE PERIOD */
#define MT9D113_IMAGE_SENSOR_FULL_HBLANKING   380
#define MT9D113_IMAGE_SENSOR_FULL_VBLANKING   93
#define MT9D113_IMAGE_SENSOR_PV_HBLANKING     550
#define MT9D113_IMAGE_SENSOR_PV_VBLANKING     91

/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
#define MT9D113_FULL_PERIOD_PIXEL_NUMS        (MT9D113_IMAGE_SENSOR_FULL_HACTIVE + MT9D113_IMAGE_SENSOR_FULL_HBLANKING) /* 53: 2234, other: 1980 */
#define MT9D113_FULL_PERIOD_LINE_NUMS         (MT9D113_IMAGE_SENSOR_FULL_VACTIVE + MT9D113_IMAGE_SENSOR_FULL_VBLANKING) /* 1293 */
#define MT9D113_PV_PERIOD_PIXEL_NUMS          (MT9D113_IMAGE_SENSOR_PV_HACTIVE + MT9D113_IMAGE_SENSOR_PV_HBLANKING) /* 1350 */
#define MT9D113_PV_PERIOD_LINE_NUMS           (MT9D113_IMAGE_SENSOR_PV_VACTIVE + MT9D113_IMAGE_SENSOR_PV_VBLANKING) /* 691 */

/* SENSOR START/END POSITION */
#define MT9D113_FULL_X_START                  8
#define MT9D113_FULL_Y_START                  6
#define MT9D113_IMAGE_SENSOR_FULL_WIDTH       (MT9D113_IMAGE_SENSOR_FULL_HACTIVE - MT9D113_FULL_X_START * 2) /* 1584 */
#define MT9D113_IMAGE_SENSOR_FULL_HEIGHT      (MT9D113_IMAGE_SENSOR_FULL_VACTIVE - MT9D113_FULL_Y_START * 2) /* 1188 */
#define MT9D113_PV_X_START                    (MT9D113_FULL_X_START / 2) /* 4 */
#define MT9D113_PV_Y_START                    (MT9D113_FULL_Y_START / 2) /* 3 */
#define MT9D113_IMAGE_SENSOR_PV_WIDTH         (MT9D113_IMAGE_SENSOR_PV_HACTIVE - MT9D113_PV_X_START * 2) /* 792 */
#define MT9D113_IMAGE_SENSOR_PV_HEIGHT        (MT9D113_IMAGE_SENSOR_PV_VACTIVE - MT9D113_PV_Y_START * 2) /* 594 */


/* AE TARGET ZERO when EV = 0. MUST BE > 40 !!! */
#define MT9D113_AE_TARGET_ZERO                67

/* ANALOG GAIN BASE */
#define MT9D113_ANALOG_GAIN_BASE              0x10 /* 1x: reg / 0x10 */

/* LIMIT OF ANALOG GAIN: if too large will cause yellow frame!!! */
#define MT9D113_MAX_ANALOG_GAIN               (MT9D113_ANALOG_GAIN_BASE * 10)

/* 50HZ/60HZ */
#define MT9D113_NUM_50HZ                      50
#define MT9D113_NUM_60HZ                      60

/* FRAME RATE UNIT */
#define MT9D113_FRAME_RATE_UNIT               10

#define MT9D113_FPS(x)                        (10 * (x))


/* SENSOR READ/WRITE ID */
#define MT9D113_SLV1_WRITE_ID                 0x78
#define MT9D113_SLV1_READ_ID                  0x79
#define MT9D113_SLV2_WRITE_ID                 0x7A
#define MT9D113_SLV2_READ_ID                  0x7B


struct MT9D113_sensor_STRUCT
{
  kal_uint32 banding;
  kal_uint32 effect;
  kal_uint32 exposure;
  kal_uint32 wb;
  kal_uint8 mirror;
  kal_bool night_mode;
  kal_bool enable_3a;
  kal_bool video_frame;
  kal_bool pv_mode;
  kal_bool video_mode;
  kal_bool initial;
  kal_bool first_pv;
  kal_uint16 normal_fps; /* normal mode min fps */
  kal_uint16 night_fps; /* night mode min fps */
  kal_uint16 Fps;
#ifndef MT9D113_HW_SCCB
  kal_uint8 write_id;
  kal_uint8 read_id;
#endif
  kal_uint32 pclk;
  kal_uint16 shutter;
  kal_uint16 pv_frame_height;
  kal_uint16 pv_line_length;
  kal_uint16 cap_frame_height;
  kal_uint16 cap_line_length;
};
extern 	struct MT9D113_sensor_STRUCT MT9D113_sensor;


//Export functions
UINT32 MT9D113Open(void);
UINT32 MT9D113GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 MT9D113GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 MT9D113Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 MT9D113FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 MT9D113Close(void);

UINT32 MT9D113YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara);


#endif /* #ifdef __SENSOR_MT9D113_H */
