/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H

typedef enum _SID130B_OP_TYPE_ {
        SID130B_MODE_NONE,
        SID130B_MODE_PREVIEW,
        SID130B_MODE_CAPTURE,
        SID130B_MODE_QCIF_VIDEO,
        SID130B_MODE_CIF_VIDEO,
        SID130B_MODE_QVGA_VIDEO
    } SID130B_OP_TYPE;

extern SID130B_OP_TYPE SID130B_g_iSID130B_Mode;

/* START GRAB PIXEL OFFSET */
#define IMAGE_SENSOR_START_GRAB_X		        2	// 0 or 1 recommended
#define IMAGE_SENSOR_START_GRAB_Y		        2	// 0 or 1 recommended

/* MAX/MIN FRAME RATE (FRAMES PER SEC.) */
#define MAX_FRAME_RATE							15		// Limitation for MPEG4 Encode Only
#define MIN_FRAME_RATE							12

/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
    #define SID130B_FULL_PERIOD_PIXEL_NUMS  (2376)  // default pixel#(w/o dummy pixels) in UXGA mode
    #define SID130B_FULL_PERIOD_LINE_NUMS   (1568)  // default line#(w/o dummy lines) in UXGA mode
    #define SID130B_PV_PERIOD_PIXEL_NUMS    (SID130B_FULL_PERIOD_PIXEL_NUMS / 2)  // default pixel#(w/o dummy pixels) in SVGA mode
    #define SID130B_PV_PERIOD_LINE_NUMS     (784)   // default line#(w/o dummy lines) in SVGA mode

    /* SENSOR EXPOSURE LINE LIMITATION */
    #define SID130B_FULL_MAX_LINES_PER_FRAME    (1568)  // QXGA mode    
    #define SID130B_FULL_EXPOSURE_LIMITATION    (SID130B_FULL_MAX_LINES_PER_FRAME)
    #define SID130B_PV_MAX_LINES_PER_FRAME      (784)  // # of lines in one XGA frame    
    #define SID130B_PV_EXPOSURE_LIMITATION      (SID130B_PV_MAX_LINES_PER_FRAME)

/* SENSOR FULL SIZE */
   #define SID130B_IMAGE_SENSOR_FULL_WIDTH	   (1600)  
   #define SID130B_IMAGE_SENSOR_FULL_HEIGHT	 (1200)    



/* SENSOR PV SIZE */
#define SID130B_IMAGE_SENSOR_PV_WIDTH   (800)   
#define SID130B_IMAGE_SENSOR_PV_HEIGHT (600)


//SENSOR 3M size
#define SID130B_IMAGE_SENSOR_3M_WIDTH 	   (2048)	  
#define SID130B_IMAGE_SENSOR_3M_HEIGHT	   (1536)


/* SENSOR READ/WRITE ID */
#define SID130B_WRITE_ID					0x6E
#define SID130B_READ_ID						0x6F

/* SENSOR CHIP VERSION */
#define SID130B_SENSOR_ID					0x1B

/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
#define SID130B_PV_PERIOD_PIXEL_NUMS          (816 + 359 + 7 + 2)/* Active + HST + Row fill+ 2 */
#define SID130B_PV_PERIOD_LINE_NUMS           (616 + 9)      /* Active + 9 */

#define SID130B_CAP_PERIOD_PIXEL_NUMS          (1630 + 359 + 7 + 2)/* Active + HST + Row fill+ 2 */
#define SID130B_CAP_PERIOD_LINE_NUMS           (1230 + 9)      /* Active + 9 */


#define SID130B_BLANK_REGISTER_LIMITATION   0xFFF

/*50Hz,60Hz*/
#define SID130B_NUM_50HZ    50 * 2
#define SID130B_NUM_60HZ    60 * 2

/* FRAME RATE UNIT */
#define SID130B_FRAME_RATE_UNIT               10

/* MAX CAMERA FRAME RATE */
#define SID130B_MAX_CAMERA_FPS         (SID130B_FRAME_RATE_UNIT * 30)


//export functions
UINT32 SID130BOpen(void);
UINT32 SID130BGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 SID130BGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 SID130BControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 SID130BFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 SID130BClose(void);

#endif /* __SENSOR_H */
