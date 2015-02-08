

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
#include "kd_camera_feature.h"

#include "mt9d113yuv_Sensor.h"
#include "mt9d113yuv_Camera_Sensor_para.h"
#include "mt9d113yuv_CameraCustomized.h"

MSDK_SENSOR_CONFIG_STRUCT MT9D113SensorConfigData;

struct MT9D113_sensor_STRUCT MT9D113_sensor;
 static kal_uint32 MT9D113_zoom_factor = 0; 


extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

kal_uint16 MT9D113_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 4, MT9D113_sensor.write_id);

}
kal_uint16 MT9D113_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2, MT9D113_sensor.write_id);

    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

static void MT9D113_write_XDMA(kal_uint16 addr, kal_uint16 para)
{
	MT9D113_write_cmos_sensor(0x098C, addr);
	MT9D113_write_cmos_sensor(0x0990, para);
}


static kal_uint16 MT9D113_read_XDMA(kal_uint16 addr)
{
  MT9D113_write_cmos_sensor(0x098C, addr);
  return MT9D113_read_cmos_sensor(0x0990);
}


static void MT9D113_burst_write_XDMA(kal_uint16 addr, const kal_uint16 *data, kal_uint16 len)
{
  kal_uint16 i, j = 0;
  
  for (i = 0; i < len; i++, j += 2)
  {
	if (!(i&7))
	{
	  j = 0;
	  MT9D113_write_cmos_sensor(0x098C, addr + i * 2);
	}
	MT9D113_write_cmos_sensor(0x0990 + j, data[i]);
  }
}


__inline static kal_uint16 MT9D113_half_adjust(kal_uint32 dividend, kal_uint32 divisor)
{
  return (dividend * 2 + divisor) / (divisor * 2); /* that is [dividend / divisor + 0.5]*/
}

static void MT9D113_set_cmd(kal_uint16 cmd)
{
  const kal_uint8 status = (cmd == 1 ? 3 : 7);
  kal_uint8 delay, i;
  
  if (MT9D113_sensor.initial) return;
  if ((cmd == 1 && MT9D113_sensor.pv_mode) || (cmd == 2 && !MT9D113_sensor.pv_mode)) return;
  for (delay = 8, i = 100; i > 0; i--) /* check if ready to set */
  {
	if (!MT9D113_read_XDMA(0xA103)) break;
	mdelay(delay);
	delay = 2;
  }
#ifdef __MT9D113_DEBUG_TRACE__
  if (!i) MT9D113_TRACE("sequencer time out");
#endif
  MT9D113_write_XDMA(0xA103, cmd);
  if (cmd >= 5) return;
  MT9D113_sensor.pv_mode = !MT9D113_sensor.pv_mode;
  for (delay = 8, i = 100; i > 0; i--) /* wait for ready */
  {
	if (MT9D113_read_XDMA(0xA104) == status) break;
	mdelay(delay);
	delay = 2;
  }
#ifdef __MT9D113_DEBUG_TRACE__
  if (!i) MT9D113_TRACE("set cmd: %d time out", cmd);
#endif
}



static void MT9D113_set_mirror(kal_uint8 mirror)
{
  kal_uint8 sensor_mirror = 0;
  
  if (MT9D113_sensor.mirror == mirror) return;
  MT9D113_sensor.mirror = mirror;
  switch (MT9D113_sensor.mirror)
  {
  case IMAGE_H_MIRROR:
    sensor_mirror = 1;
    break;
  case IMAGE_V_MIRROR:
    sensor_mirror = 2;
    break;
  case IMAGE_HV_MIRROR:
    sensor_mirror = 3;
    break;
  default:
    break;
  }

  MT9D113_write_XDMA(0x2717, 0x046C|sensor_mirror); /* preview read mode */
  MT9D113_write_XDMA(0x272D, 0x0024|sensor_mirror); /* snapshot/video read mode */
  MT9D113_set_cmd(6); /* refresh mode */
}

static void MT9D113_set_pv_dummy(kal_uint16 dummy_pixel, kal_uint16 dummy_line)
{
  const kal_uint16 frame_height = MT9D113_PV_PERIOD_LINE_NUMS + dummy_line;
  const kal_uint16 line_length = MT9D113_PV_PERIOD_PIXEL_NUMS + dummy_pixel;
  
  if (MT9D113_sensor.pv_frame_height == frame_height && MT9D113_sensor.pv_line_length == line_length)
  		return;
  
  MT9D113_sensor.pv_frame_height = frame_height;
  MT9D113_sensor.pv_line_length = line_length;
  MT9D113_write_XDMA(0x271F, frame_height); /* preview frame length */
  MT9D113_write_XDMA(0x2721, line_length); /* preview line length */
  MT9D113_write_XDMA(0x2411, MT9D113_half_adjust(MT9D113_sensor.pclk / 2, line_length * MT9D113_NUM_60HZ * 2));/* preview fd_r9_step_f60 */
  MT9D113_write_XDMA(0x2413, MT9D113_half_adjust(MT9D113_sensor.pclk / 2, line_length * MT9D113_NUM_50HZ * 2));/* preview fd_r9_step_f50 */
  MT9D113_set_cmd(6); /* refresh mode */
}

static void MT9D113_set_cap_dummy(kal_uint16 dummy_pixel, kal_uint16 dummy_line)
{
  const kal_uint16 frame_height = MT9D113_FULL_PERIOD_LINE_NUMS + dummy_line;
  const kal_uint16 line_length = MT9D113_FULL_PERIOD_PIXEL_NUMS + dummy_pixel;
  
  if (MT9D113_sensor.cap_frame_height == frame_height && MT9D113_sensor.cap_line_length == line_length)
  		return;
  
  MT9D113_sensor.cap_frame_height = frame_height;
  MT9D113_sensor.cap_line_length = line_length;
  MT9D113_write_XDMA(0x2735, frame_height); /* snapshot/video frame length */
  MT9D113_write_XDMA(0x2737, line_length); /* snapshot/video line length */
  MT9D113_write_XDMA(0x2415, MT9D113_half_adjust(MT9D113_sensor.pclk / 2, line_length * MT9D113_NUM_60HZ * 2));/* snapshot/video fd_r9_step_f60 */
  MT9D113_write_XDMA(0x2417, MT9D113_half_adjust(MT9D113_sensor.pclk / 2, line_length * MT9D113_NUM_50HZ * 2));/* snapshot/video fd_r9_step_f50 */
  MT9D113_set_cmd(6); /* refresh mode */
}


static kal_uint32 MT9D113_set_param_3a_enable(kal_uint32 para)
{
  kal_uint8 seq_preview_1_ae, seq_preview_1_fd, seq_preview_1_awb, seq_preview_1_hg;
  
  if (MT9D113_sensor.enable_3a == para) return KAL_TRUE;
  MT9D113_sensor.enable_3a = para;
  if (MT9D113_sensor.enable_3a)
  {
    seq_preview_1_ae = seq_preview_1_fd = 2;
    seq_preview_1_awb = seq_preview_1_hg = 1;
  }
  else
  {
    seq_preview_1_ae = seq_preview_1_fd = seq_preview_1_awb = seq_preview_1_hg = 0;
  }
  MT9D113_write_XDMA(0xA11D, seq_preview_1_ae);
  MT9D113_write_XDMA(0xA11E, seq_preview_1_fd);
  MT9D113_write_XDMA(0xA11F, seq_preview_1_awb);
  MT9D113_write_XDMA(0xA120, seq_preview_1_hg);
  MT9D113_set_cmd(6); /* refresh mode */
  
  return KAL_TRUE;
}


static void MT9D113_cal_fps(void)
{
	const kal_uint16 dummy_pixel = MT9D113_sensor.pv_line_length - MT9D113_PV_PERIOD_PIXEL_NUMS;
	const kal_uint8 banding = MT9D113_sensor.banding == AE_FLICKER_MODE_50HZ ? MT9D113_NUM_50HZ : MT9D113_NUM_60HZ;
	const kal_uint16 pv_max_fps = MT9D113_sensor.pclk * MT9D113_FRAME_RATE_UNIT / (2 * MT9D113_sensor.pv_line_length * MT9D113_sensor.pv_frame_height);
	kal_uint16 pv_min_fps =  MT9D113_sensor.night_mode ? MT9D113_sensor.night_fps : MT9D113_sensor.normal_fps;
	kal_uint16 max_exposure_lines;

	MT9D113_TRACE("MT9D113_cal_fps MT9D113_sensor.video_mode %d\n",MT9D113_sensor.video_mode);

	if (pv_min_fps > pv_max_fps) pv_min_fps = pv_max_fps;
	if (MT9D113_sensor.video_mode) /* fix frame rate when video mode */
	{
		MT9D113_TRACE("MT9D113_cal_fps MT9D113_sensor.video_frame %d\n",MT9D113_sensor.video_frame);
		pv_min_fps =  MT9D113_sensor.video_frame ? MT9D113_sensor.night_fps : MT9D113_sensor.normal_fps;

		MT9D113_TRACE("MT9D113_cal_fps pv_min_fps %d\n",pv_min_fps);
		max_exposure_lines = MT9D113_sensor.pclk * MT9D113_FRAME_RATE_UNIT / (2 * pv_min_fps * MT9D113_sensor.pv_line_length);
		MT9D113_set_pv_dummy(dummy_pixel, max_exposure_lines - MT9D113_PV_PERIOD_LINE_NUMS);
	}
	MT9D113_write_XDMA(0xA20C, banding * 2 * MT9D113_FRAME_RATE_UNIT / pv_min_fps);
	MT9D113_set_cmd(5); /* refresh */

}




static void MT9D113_gamma_setting(void)
{
  MT9D113_write_cmos_sensor(0x098C, 0xAB3C); /* HG_GAMMA_TABLE_A_0 */
  MT9D113_write_cmos_sensor(0x0990, 0x0000); /* #24 */
  MT9D113_write_cmos_sensor(0x098C, 0xAB3D); /* HG_GAMMA_TABLE_A_1 */
  MT9D113_write_cmos_sensor(0x0990, 0x0010); /* #23 */
  MT9D113_write_cmos_sensor(0x098C, 0xAB3E); /* HG_GAMMA_TABLE_A_2 */
  MT9D113_write_cmos_sensor(0x0990, 0x002E); /* #22 */
  MT9D113_write_cmos_sensor(0x098C, 0xAB3F); /* HG_GAMMA_TABLE_A_3 */
  MT9D113_write_cmos_sensor(0x0990, 0x004C); /* #21 */
  MT9D113_write_cmos_sensor(0x098C, 0xAB40); /* HG_GAMMA_TABLE_A_4 */
  MT9D113_write_cmos_sensor(0x0990, 0x0078); /* #20 */
  MT9D113_write_cmos_sensor(0x098C, 0xAB41); /* HG_GAMMA_TABLE_A_5 */
  MT9D113_write_cmos_sensor(0x0990, 0x0098); /* #19 */
  MT9D113_write_cmos_sensor(0x098C, 0xAB42); /* HG_GAMMA_TABLE_A_6 */
  MT9D113_write_cmos_sensor(0x0990, 0x00B0);
  MT9D113_write_cmos_sensor(0x098C, 0xAB43); /* HG_GAMMA_TABLE_A_7 */
  MT9D113_write_cmos_sensor(0x0990, 0x00C1);
  MT9D113_write_cmos_sensor(0x098C, 0xAB44); /* HG_GAMMA_TABLE_A_8 */
  MT9D113_write_cmos_sensor(0x0990, 0x00CF);
  MT9D113_write_cmos_sensor(0x098C, 0xAB45); /* HG_GAMMA_TABLE_A_9 */
  MT9D113_write_cmos_sensor(0x0990, 0x00D9);
  MT9D113_write_cmos_sensor(0x098C, 0xAB46); /* HG_GAMMA_TABLE_A_10 */
  MT9D113_write_cmos_sensor(0x0990, 0x00E1);
  MT9D113_write_cmos_sensor(0x098C, 0xAB47); /* HG_GAMMA_TABLE_A_11 */
  MT9D113_write_cmos_sensor(0x0990, 0x00E8);
  MT9D113_write_cmos_sensor(0x098C, 0xAB48); /* HG_GAMMA_TABLE_A_12 */
  MT9D113_write_cmos_sensor(0x0990, 0x00EE);
  MT9D113_write_cmos_sensor(0x098C, 0xAB49); /* HG_GAMMA_TABLE_A_13 */
  MT9D113_write_cmos_sensor(0x0990, 0x00F2);
  MT9D113_write_cmos_sensor(0x098C, 0xAB4A); /* HG_GAMMA_TABLE_A_14 */
  MT9D113_write_cmos_sensor(0x0990, 0x00F6);
  MT9D113_write_cmos_sensor(0x098C, 0xAB4B); /* HG_GAMMA_TABLE_A_15 */
  MT9D113_write_cmos_sensor(0x0990, 0x00F9);
  MT9D113_write_cmos_sensor(0x098C, 0xAB4C); /* HG_GAMMA_TABLE_A_16 */
  MT9D113_write_cmos_sensor(0x0990, 0x00FB);
  MT9D113_write_cmos_sensor(0x098C, 0xAB4D); /* HG_GAMMA_TABLE_A_17 */
  MT9D113_write_cmos_sensor(0x0990, 0x00FD);
  MT9D113_write_cmos_sensor(0x098C, 0xAB4E); /* HG_GAMMA_TABLE_A_18 */
  MT9D113_write_cmos_sensor(0x0990, 0x00FF);

  MT9D113_write_cmos_sensor(0x098C, 0xAB4F); /* HG_GAMMA_TABLE_B_0 */
  MT9D113_write_cmos_sensor(0x0990, 0x0000); /* #24 */
  MT9D113_write_cmos_sensor(0x098C, 0xAB50); /* HG_GAMMA_TABLE_B_1 */
  MT9D113_write_cmos_sensor(0x0990, 0x0002); /* #23 */
  MT9D113_write_cmos_sensor(0x098C, 0xAB51); /* HG_GAMMA_TABLE_B_2 */
  MT9D113_write_cmos_sensor(0x0990, 0x0010); /* #22 */
  MT9D113_write_cmos_sensor(0x098C, 0xAB52); /* HG_GAMMA_TABLE_B_3 */
  MT9D113_write_cmos_sensor(0x0990, 0x002E); /* #21 */
  MT9D113_write_cmos_sensor(0x098C, 0xAB53); /* HG_GAMMA_TABLE_B_4 */
  MT9D113_write_cmos_sensor(0x0990, 0x005A); /* #20 */
  MT9D113_write_cmos_sensor(0x098C, 0xAB54); /* HG_GAMMA_TABLE_B_5 */
  MT9D113_write_cmos_sensor(0x0990, 0x006A); /* #19 */
  MT9D113_write_cmos_sensor(0x098C, 0xAB55); /* HG_GAMMA_TABLE_B_6 */
  MT9D113_write_cmos_sensor(0x0990, 0x0080);
  MT9D113_write_cmos_sensor(0x098C, 0xAB56); /* HG_GAMMA_TABLE_B_7 */
  MT9D113_write_cmos_sensor(0x0990, 0x0091);
  MT9D113_write_cmos_sensor(0x098C, 0xAB57); /* HG_GAMMA_TABLE_B_8 */
  MT9D113_write_cmos_sensor(0x0990, 0x00A1);
  MT9D113_write_cmos_sensor(0x098C, 0xAB58); /* HG_GAMMA_TABLE_B_9 */
  MT9D113_write_cmos_sensor(0x0990, 0x00AF);
  MT9D113_write_cmos_sensor(0x098C, 0xAB59); /* HG_GAMMA_TABLE_B_10 */
  MT9D113_write_cmos_sensor(0x0990, 0x00BB);
  MT9D113_write_cmos_sensor(0x098C, 0xAB5A); /* HG_GAMMA_TABLE_B_11 */
  MT9D113_write_cmos_sensor(0x0990, 0x00C6);
  MT9D113_write_cmos_sensor(0x098C, 0xAB5B); /* HG_GAMMA_TABLE_B_12 */
  MT9D113_write_cmos_sensor(0x0990, 0x00D0);
  MT9D113_write_cmos_sensor(0x098C, 0xAB5C); /* HG_GAMMA_TABLE_B_13 */
  MT9D113_write_cmos_sensor(0x0990, 0x00D9);
  MT9D113_write_cmos_sensor(0x098C, 0xAB5D); /* HG_GAMMA_TABLE_B_14 */
  MT9D113_write_cmos_sensor(0x0990, 0x00E2);
  MT9D113_write_cmos_sensor(0x098C, 0xAB5E); /* HG_GAMMA_TABLE_B_15 */
  MT9D113_write_cmos_sensor(0x0990, 0x00EA);
  MT9D113_write_cmos_sensor(0x098C, 0xAB5F); /* HG_GAMMA_TABLE_B_16 */
  MT9D113_write_cmos_sensor(0x0990, 0x00F1);
  MT9D113_write_cmos_sensor(0x098C, 0xAB60); /* HG_GAMMA_TABLE_B_17 */
  MT9D113_write_cmos_sensor(0x0990, 0x00F9);
  MT9D113_write_cmos_sensor(0x098C, 0xAB61); /* HG_GAMMA_TABLE_B_18 */
  MT9D113_write_cmos_sensor(0x0990, 0x00FF);
}
static void MT9D113_low_light(void)
{
  MT9D113_write_cmos_sensor(0x098C, 0x2B1B); /* HG_BRIGHTNESSMETRIC */
  MT9D113_write_cmos_sensor(0x0990, 0x392D);
  MT9D113_write_cmos_sensor(0x098C, 0x2B1D); /* HG_LASTBRIGHTNESSMETRIC */
  MT9D113_write_cmos_sensor(0x0990, 0x392D);
  MT9D113_write_cmos_sensor(0x098C, 0xAB1F); /* HG_LLMODE */
  MT9D113_write_cmos_sensor(0x0990, 0x00C5);
  /* saturation setting: 0xAB20*/
  MT9D113_write_cmos_sensor(0x098C, 0xAB20); /* HG_LL_SAT1 */
  MT9D113_write_cmos_sensor(0x0990, 0x003B);

  MT9D113_write_cmos_sensor(0x098C, 0xAB21); /* HG_LL_INTERPTHRESH1 */
  MT9D113_write_cmos_sensor(0x0990, 0x0010);
  MT9D113_write_cmos_sensor(0x098C, 0xAB22); /* HG_LL_APCORR1 */
  MT9D113_write_cmos_sensor(0x0990, 0x0004);
  MT9D113_write_cmos_sensor(0x098C, 0xAB23); /* HG_LL_APTHRESH1 */
  MT9D113_write_cmos_sensor(0x0990, 0x0004);
  MT9D113_write_cmos_sensor(0x098C, 0xAB24); /* HG_LL_SAT2 */
  MT9D113_write_cmos_sensor(0x0990, 0x0000);
  MT9D113_write_cmos_sensor(0x098C, 0xAB25); /* HG_LL_INTERPTHRESH2 */
  MT9D113_write_cmos_sensor(0x0990, 0x0032);
  MT9D113_write_cmos_sensor(0x098C, 0xAB26); /* HG_LL_APCORR2 */
  MT9D113_write_cmos_sensor(0x0990, 0x0000);
  MT9D113_write_cmos_sensor(0x098C, 0xAB27); /* HG_LL_APTHRESH2 */
  MT9D113_write_cmos_sensor(0x0990, 0x0008);
  MT9D113_write_cmos_sensor(0x098C, 0x2B28); /* HG_LL_BRIGHTNESSSTART */
  MT9D113_write_cmos_sensor(0x0990, 0x1E14);
  MT9D113_write_cmos_sensor(0x098C, 0x2B2A); /* HG_LL_BRIGHTNESSSTOP */
  MT9D113_write_cmos_sensor(0x0990, 0x3A98);
  MT9D113_write_cmos_sensor(0x098C, 0xAB2C); /* HG_NR_START_R */
  MT9D113_write_cmos_sensor(0x0990, 0x0001);
  MT9D113_write_cmos_sensor(0x098C, 0xAB2D); /* HG_NR_START_G */
  MT9D113_write_cmos_sensor(0x0990, 0x0001);
  MT9D113_write_cmos_sensor(0x098C, 0xAB2E); /* HG_NR_START_B */
  MT9D113_write_cmos_sensor(0x0990, 0x0001);
  MT9D113_write_cmos_sensor(0x098C, 0xAB2F); /* HG_NR_START_OL */
  MT9D113_write_cmos_sensor(0x0990, 0x0001);
  MT9D113_write_cmos_sensor(0x098C, 0xAB30); /* HG_NR_STOP_R */
  MT9D113_write_cmos_sensor(0x0990, 0x00F0);
  MT9D113_write_cmos_sensor(0x098C, 0xAB31); /* HG_NR_STOP_G */
  MT9D113_write_cmos_sensor(0x0990, 0x00F0);
  MT9D113_write_cmos_sensor(0x098C, 0xAB32); /* HG_NR_STOP_B */
  MT9D113_write_cmos_sensor(0x0990, 0x00F0);
  MT9D113_write_cmos_sensor(0x098C, 0xAB33); /* HG_NR_STOP_OL */
  MT9D113_write_cmos_sensor(0x0990, 0x00F0);
  MT9D113_write_cmos_sensor(0x098C, 0xAB36); /* HG_CLUSTERDC_TH */
  MT9D113_write_cmos_sensor(0x0990, 0x002D);
  MT9D113_write_cmos_sensor(0x098C, 0x2B62); /* HG_FTB_START_BM */
  MT9D113_write_cmos_sensor(0x0990, 0xFFFF);
  MT9D113_write_cmos_sensor(0x098C, 0x2B64); /* HG_FTB_STOP_BM */
  MT9D113_write_cmos_sensor(0x0990, 0xFFFF);
  MT9D113_write_cmos_sensor(0x098C, 0x2B66); /* HG_CLUSTER_DC_BM */
  MT9D113_write_cmos_sensor(0x0990, 0x3A98);
  MT9D113_write_cmos_sensor(0x098C, 0x275F); /* MODE_COMMONMODESETTINGS_BRIGHT_COLOR_KILL */
  MT9D113_write_cmos_sensor(0x0990, 0x0595);
  MT9D113_write_cmos_sensor(0x098C, 0x2761); /* MODE_COMMONMODESETTINGS_DARK_COLOR_KILL */
  MT9D113_write_cmos_sensor(0x0990, 0x0080);
  MT9D113_write_cmos_sensor(0x098C, 0xA765); /* MODE_COMMONMODESETTINGS_FILTER_MODE */
  MT9D113_write_cmos_sensor(0x0990, 0x00A4);
  MT9D113_write_cmos_sensor(0x098C, 0xAB34); /* HG_NR_GAINSTART */
  MT9D113_write_cmos_sensor(0x0990, 0x0008);
  MT9D113_write_cmos_sensor(0x098C, 0xAB35); /* HG_NR_GAINSTOP */
  MT9D113_write_cmos_sensor(0x0990, 0x0078);
}
static void MT9D113_awb_ccms(void)
{
  MT9D113_write_cmos_sensor(0x098C, 0x2306); /* AWB_CCM_L_0 */
  MT9D113_write_cmos_sensor(0x0990, 0x0180);
  MT9D113_write_cmos_sensor(0x098C, 0x2308); /* AWB_CCM_L_1 */
  MT9D113_write_cmos_sensor(0x0990, 0xFF00);
  MT9D113_write_cmos_sensor(0x098C, 0x230A); /* AWB_CCM_L_2 */
  MT9D113_write_cmos_sensor(0x0990, 0x0080);
  MT9D113_write_cmos_sensor(0x098C, 0x230C); /* AWB_CCM_L_3 */
  MT9D113_write_cmos_sensor(0x0990, 0xFF66);
  MT9D113_write_cmos_sensor(0x098C, 0x230E); /* AWB_CCM_L_4 */
  MT9D113_write_cmos_sensor(0x0990, 0x0180);
  MT9D113_write_cmos_sensor(0x098C, 0x2310); /* AWB_CCM_L_5 */
  MT9D113_write_cmos_sensor(0x0990, 0xFFEE);
  MT9D113_write_cmos_sensor(0x098C, 0x2312); /* AWB_CCM_L_6 */
  MT9D113_write_cmos_sensor(0x0990, 0xFFCD);
  MT9D113_write_cmos_sensor(0x098C, 0x2314); /* AWB_CCM_L_7 */
  MT9D113_write_cmos_sensor(0x0990, 0xFECD);
  MT9D113_write_cmos_sensor(0x098C, 0x2316); /* AWB_CCM_L_8 */
  MT9D113_write_cmos_sensor(0x0990, 0x019A);
  MT9D113_write_cmos_sensor(0x098C, 0x2318); /* AWB_CCM_L_9 */
  MT9D113_write_cmos_sensor(0x0990, 0x0020);
  MT9D113_write_cmos_sensor(0x098C, 0x231A); /* AWB_CCM_L_10 */
  MT9D113_write_cmos_sensor(0x0990, 0x0033);
  MT9D113_write_cmos_sensor(0x098C, 0x231C); /* AWB_CCM_RL_0 */
  MT9D113_write_cmos_sensor(0x0990, 0x0100);
  MT9D113_write_cmos_sensor(0x098C, 0x231E); /* AWB_CCM_RL_1 */
  MT9D113_write_cmos_sensor(0x0990, 0xFF9A);
  MT9D113_write_cmos_sensor(0x098C, 0x2320); /* AWB_CCM_RL_2 */
  MT9D113_write_cmos_sensor(0x0990, 0x0000);
  MT9D113_write_cmos_sensor(0x098C, 0x2322); /* AWB_CCM_RL_3 */
  MT9D113_write_cmos_sensor(0x0990, 0x004D);
  MT9D113_write_cmos_sensor(0x098C, 0x2324); /* AWB_CCM_RL_4 */
  MT9D113_write_cmos_sensor(0x0990, 0xFFCD);
  MT9D113_write_cmos_sensor(0x098C, 0x2326); /* AWB_CCM_RL_5 */
  MT9D113_write_cmos_sensor(0x0990, 0xFFB8);
  MT9D113_write_cmos_sensor(0x098C, 0x2328); /* AWB_CCM_RL_6 */
  MT9D113_write_cmos_sensor(0x0990, 0x004D);
  MT9D113_write_cmos_sensor(0x098C, 0x232A); /* AWB_CCM_RL_7 */
  MT9D113_write_cmos_sensor(0x0990, 0x0080);
  MT9D113_write_cmos_sensor(0x098C, 0x232C); /* AWB_CCM_RL_8 */
  MT9D113_write_cmos_sensor(0x0990, 0xFF66);
  MT9D113_write_cmos_sensor(0x098C, 0x232E); /* AWB_CCM_RL_9 */
  MT9D113_write_cmos_sensor(0x0990, 0x0010);
  MT9D113_write_cmos_sensor(0x098C, 0x2330); /* AWB_CCM_RL_10 */
  MT9D113_write_cmos_sensor(0x0990, 0xFFF7);
  MT9D113_write_cmos_sensor(0x098C, 0xA363); /* AWB_TG_MIN0 */
  MT9D113_write_cmos_sensor(0x0990, 0x00D2);
  MT9D113_write_cmos_sensor(0x098C, 0xA364); /* AWB_TG_MAX0 */
  MT9D113_write_cmos_sensor(0x0990, 0x00EE);
  
  MT9D113_write_cmos_sensor(0x3244, 0x0302); /* AWB_CONFIG4: change AWB: 0x0328 */
  MT9D113_write_cmos_sensor(0x323E, 0xC22C); /* AWB_CONFIG1 */
}

static void MT9D113_LSC_setting(void)
{
  /* [Register Log 12/22/08 16:42:15] */
  MT9D113_write_cmos_sensor(0x3658, 0x00B0); /* P_RD_P0Q0 */
  MT9D113_write_cmos_sensor(0x365A, 0xB3A8); /* P_RD_P0Q1 */
  MT9D113_write_cmos_sensor(0x365C, 0x4992); /* P_RD_P0Q2 */
  MT9D113_write_cmos_sensor(0x365E, 0xA74D); /* P_RD_P0Q3 */
  MT9D113_write_cmos_sensor(0x3660, 0x9C93); /* P_RD_P0Q4 */
  MT9D113_write_cmos_sensor(0x3680, 0x872D); /* P_RD_P1Q0 */
  MT9D113_write_cmos_sensor(0x3682, 0x1E50); /* P_RD_P1Q1 */
  MT9D113_write_cmos_sensor(0x3684, 0xD7D0); /* P_RD_P1Q2 */
  MT9D113_write_cmos_sensor(0x3686, 0xE352); /* P_RD_P1Q3 */
  MT9D113_write_cmos_sensor(0x3688, 0x2832); /* P_RD_P1Q4 */
  MT9D113_write_cmos_sensor(0x36A8, 0x27B3); /* P_RD_P2Q0 */
  MT9D113_write_cmos_sensor(0x36AA, 0x5FF0); /* P_RD_P2Q1 */
  MT9D113_write_cmos_sensor(0x36AC, 0x7C70); /* P_RD_P2Q2 */
  MT9D113_write_cmos_sensor(0x36AE, 0xD894); /* P_RD_P2Q3 */
  MT9D113_write_cmos_sensor(0x36B0, 0xEDB7); /* P_RD_P2Q4 */
  MT9D113_write_cmos_sensor(0x36D0, 0xE6CF); /* P_RD_P3Q0 */
  MT9D113_write_cmos_sensor(0x36D2, 0x9731); /* P_RD_P3Q1 */
  MT9D113_write_cmos_sensor(0x36D4, 0x69D4); /* P_RD_P3Q2 */
  MT9D113_write_cmos_sensor(0x36D6, 0x2B34); /* P_RD_P3Q3 */
  MT9D113_write_cmos_sensor(0x36D8, 0x9BB6); /* P_RD_P3Q4 */
  MT9D113_write_cmos_sensor(0x36F8, 0xB714); /* P_RD_P4Q0 */
  MT9D113_write_cmos_sensor(0x36FA, 0xC9F4); /* P_RD_P4Q1 */
  MT9D113_write_cmos_sensor(0x36FC, 0xA7F9); /* P_RD_P4Q2 */
  MT9D113_write_cmos_sensor(0x36FE, 0x39D8); /* P_RD_P4Q3 */
  MT9D113_write_cmos_sensor(0x3700, 0x2ADC); /* P_RD_P4Q4 */
  MT9D113_write_cmos_sensor(0x364E, 0x0630); /* P_GR_P0Q0 */
  MT9D113_write_cmos_sensor(0x3650, 0x0DEB); /* P_GR_P0Q1 */
  MT9D113_write_cmos_sensor(0x3652, 0x1772); /* P_GR_P0Q2 */
  MT9D113_write_cmos_sensor(0x3654, 0x824E); /* P_GR_P0Q3 */
  MT9D113_write_cmos_sensor(0x3656, 0x99F3); /* P_GR_P0Q4 */
  MT9D113_write_cmos_sensor(0x3676, 0xE98C); /* P_GR_P1Q0 */
  MT9D113_write_cmos_sensor(0x3678, 0x8230); /* P_GR_P1Q1 */
  MT9D113_write_cmos_sensor(0x367A, 0x81B1); /* P_GR_P1Q2 */
  MT9D113_write_cmos_sensor(0x367C, 0x6D71); /* P_GR_P1Q3 */
  MT9D113_write_cmos_sensor(0x367E, 0x7CF2); /* P_GR_P1Q4 */
  MT9D113_write_cmos_sensor(0x369E, 0x0B53); /* P_GR_P2Q0 */
  MT9D113_write_cmos_sensor(0x36A0, 0x2370); /* P_GR_P2Q1 */
  MT9D113_write_cmos_sensor(0x36A2, 0xCA74); /* P_GR_P2Q2 */
  MT9D113_write_cmos_sensor(0x36A4, 0x9774); /* P_GR_P2Q3 */
  MT9D113_write_cmos_sensor(0x36A6, 0x9234); /* P_GR_P2Q4 */
  MT9D113_write_cmos_sensor(0x36C6, 0xCD6F); /* P_GR_P3Q0 */
  MT9D113_write_cmos_sensor(0x36C8, 0x9FD0); /* P_GR_P3Q1 */
  MT9D113_write_cmos_sensor(0x36CA, 0x70D4); /* P_GR_P3Q2 */
  MT9D113_write_cmos_sensor(0x36CC, 0x64F4); /* P_GR_P3Q3 */
  MT9D113_write_cmos_sensor(0x36CE, 0x98F6); /* P_GR_P3Q4 */
  MT9D113_write_cmos_sensor(0x36EE, 0xA914); /* P_GR_P4Q0 */
  MT9D113_write_cmos_sensor(0x36F0, 0x98B4); /* P_GR_P4Q1 */
  MT9D113_write_cmos_sensor(0x36F2, 0xA138); /* P_GR_P4Q2 */
  MT9D113_write_cmos_sensor(0x36F4, 0x77D7); /* P_GR_P4Q3 */
  MT9D113_write_cmos_sensor(0x36F6, 0x4E5B); /* P_GR_P4Q4 */
  MT9D113_write_cmos_sensor(0x3662, 0x00D0); /* P_BL_P0Q0 */
  MT9D113_write_cmos_sensor(0x3664, 0x04E9); /* P_BL_P0Q1 */
  MT9D113_write_cmos_sensor(0x3666, 0x0572); /* P_BL_P0Q2 */
  MT9D113_write_cmos_sensor(0x3668, 0x28AC); /* P_BL_P0Q3 */
  MT9D113_write_cmos_sensor(0x366A, 0x84D3); /* P_BL_P0Q4 */
  MT9D113_write_cmos_sensor(0x368A, 0x90ED); /* P_BL_P1Q0 */
  MT9D113_write_cmos_sensor(0x368C, 0xDA0F); /* P_BL_P1Q1 */
  MT9D113_write_cmos_sensor(0x368E, 0xD42F); /* P_BL_P1Q2 */
  MT9D113_write_cmos_sensor(0x3690, 0x6691); /* P_BL_P1Q3 */
  MT9D113_write_cmos_sensor(0x3692, 0x5430); /* P_BL_P1Q4 */
  MT9D113_write_cmos_sensor(0x36B2, 0x0A13); /* P_BL_P2Q0 */
  MT9D113_write_cmos_sensor(0x36B4, 0x2A0E); /* P_BL_P2Q1 */
  MT9D113_write_cmos_sensor(0x36B6, 0xC054); /* P_BL_P2Q2 */
  MT9D113_write_cmos_sensor(0x36B8, 0x9513); /* P_BL_P2Q3 */
  MT9D113_write_cmos_sensor(0x36BA, 0x8635); /* P_BL_P2Q4 */
  MT9D113_write_cmos_sensor(0x36DA, 0x7F2D); /* P_BL_P3Q0 */
  MT9D113_write_cmos_sensor(0x36DC, 0xE690); /* P_BL_P3Q1 */
  MT9D113_write_cmos_sensor(0x36DE, 0x1DB3); /* P_BL_P3Q2 */
  MT9D113_write_cmos_sensor(0x36E0, 0x63D4); /* P_BL_P3Q3 */
  MT9D113_write_cmos_sensor(0x36E2, 0x09F3); /* P_BL_P3Q4 */
  MT9D113_write_cmos_sensor(0x3702, 0xB634); /* P_BL_P4Q0 */
  MT9D113_write_cmos_sensor(0x3704, 0xAFF3); /* P_BL_P4Q1 */
  MT9D113_write_cmos_sensor(0x3706, 0xA8D8); /* P_BL_P4Q2 */
  MT9D113_write_cmos_sensor(0x3708, 0x29B7); /* P_BL_P4Q3 */
  MT9D113_write_cmos_sensor(0x370A, 0x5F9B); /* P_BL_P4Q4 */
  MT9D113_write_cmos_sensor(0x366C, 0x0170); /* P_GB_P0Q0 */
  MT9D113_write_cmos_sensor(0x366E, 0x034A); /* P_GB_P0Q1 */
  MT9D113_write_cmos_sensor(0x3670, 0x1192); /* P_GB_P0Q2 */
  MT9D113_write_cmos_sensor(0x3672, 0xBACD); /* P_GB_P0Q3 */
  MT9D113_write_cmos_sensor(0x3674, 0x9EB3); /* P_GB_P0Q4 */
  MT9D113_write_cmos_sensor(0x3694, 0xCAAC); /* P_GB_P1Q0 */
  MT9D113_write_cmos_sensor(0x3696, 0x7A6F); /* P_GB_P1Q1 */
  MT9D113_write_cmos_sensor(0x3698, 0xB210); /* P_GB_P1Q2 */
  MT9D113_write_cmos_sensor(0x369A, 0xF191); /* P_GB_P1Q3 */
  MT9D113_write_cmos_sensor(0x369C, 0x02B2); /* P_GB_P1Q4 */
  MT9D113_write_cmos_sensor(0x36BC, 0x0FF3); /* P_GB_P2Q0 */
  MT9D113_write_cmos_sensor(0x36BE, 0x0B6F); /* P_GB_P2Q1 */
  MT9D113_write_cmos_sensor(0x36C0, 0xD794); /* P_GB_P2Q2 */
  MT9D113_write_cmos_sensor(0x36C2, 0xD813); /* P_GB_P2Q3 */
  MT9D113_write_cmos_sensor(0x36C4, 0xCED2); /* P_GB_P2Q4 */
  MT9D113_write_cmos_sensor(0x36E4, 0x974D); /* P_GB_P3Q0 */
  MT9D113_write_cmos_sensor(0x36E6, 0x1991); /* P_GB_P3Q1 */
  MT9D113_write_cmos_sensor(0x36E8, 0x2074); /* P_GB_P3Q2 */
  MT9D113_write_cmos_sensor(0x36EA, 0xE054); /* P_GB_P3Q3 */
  MT9D113_write_cmos_sensor(0x36EC, 0x9FB5); /* P_GB_P3Q4 */
  MT9D113_write_cmos_sensor(0x370C, 0xB254); /* P_GB_P4Q0 */
  MT9D113_write_cmos_sensor(0x370E, 0xA493); /* P_GB_P4Q1 */
  MT9D113_write_cmos_sensor(0x3710, 0x9B18); /* P_GB_P4Q2 */
  MT9D113_write_cmos_sensor(0x3712, 0x3C37); /* P_GB_P4Q3 */
  MT9D113_write_cmos_sensor(0x3714, 0x43FB); /* P_GB_P4Q4 */
  MT9D113_write_cmos_sensor(0x3644, 0x0320); /* POLY_ORIGIN_C */
  MT9D113_write_cmos_sensor(0x3642, 0x0258); /* POLY_ORIGIN_R */
  MT9D113_write_cmos_sensor(0x3210, 0x01B8); /* COLOR_PIPELINE_CONTROL */
  MT9D113_write_cmos_sensor(0x3644, 0x0324); /* POLY_ORIGIN_C */
  MT9D113_write_cmos_sensor(0x3642, 0x025C); /* POLY_ORIGIN_R */
}




static void MT9D113_config_context(void)
{
  /* Register wizard */
  MT9D113_write_cmos_sensor(0x98C, 0x2703); /* Output Width(A) */
  MT9D113_write_cmos_sensor(0x990, 0x0320); /* 800 */
  MT9D113_write_cmos_sensor(0x98C, 0x2705); /* Output Height(A) */
  MT9D113_write_cmos_sensor(0x990, 0x0258); /* 600 */
  MT9D113_write_cmos_sensor(0x98C, 0x2707); /* Output Width(B) */
  MT9D113_write_cmos_sensor(0x990, 0x0640); /* 1600 */
  MT9D113_write_cmos_sensor(0x98C, 0x2709); /* Output Height(B) */
  MT9D113_write_cmos_sensor(0x990, 0x04B0); /* 1200 */
  MT9D113_write_cmos_sensor(0x98C, 0x270D); /* Row Start(A) */
  MT9D113_write_cmos_sensor(0x990, 0x0000); /* 0 */
  MT9D113_write_cmos_sensor(0x98C, 0x270F); /* Column Start(A) */
  MT9D113_write_cmos_sensor(0x990, 0x0000); /* 0 */
  MT9D113_write_cmos_sensor(0x98C, 0x2711); /* Row End(A) */
  MT9D113_write_cmos_sensor(0x990, 0x04BD); /* 1213 */
  MT9D113_write_cmos_sensor(0x98C, 0x2713); /* Column End(A) */
  MT9D113_write_cmos_sensor(0x990, 0x064D); /* 1613 */
  MT9D113_write_cmos_sensor(0x98C, 0x2715); /* Row Speed(A) */
  MT9D113_write_cmos_sensor(0x990, 0x0111); /* 273 */
  MT9D113_write_cmos_sensor(0x98C, 0x2719); /* sensor_fine_correction(A) */
  MT9D113_write_cmos_sensor(0x990, 0x005A); /* 90 */
  MT9D113_write_cmos_sensor(0x98C, 0x271B); /* sensor_fine_IT_min(A) */
  MT9D113_write_cmos_sensor(0x990, 0x01BE); /* 446 */
  MT9D113_write_cmos_sensor(0x98C, 0x271D); /* sensor_fine_IT_max_margin(A) */
  MT9D113_write_cmos_sensor(0x990, 0x0131); /* 305 */
  MT9D113_write_cmos_sensor(0x98C, 0x2723); /* Row Start(B) */
  MT9D113_write_cmos_sensor(0x990, 0x0004); /* 4 */
  MT9D113_write_cmos_sensor(0x98C, 0x2725); /* Column Start(B) */
  MT9D113_write_cmos_sensor(0x990, 0x0004); /* 4 */
  MT9D113_write_cmos_sensor(0x98C, 0x2727); /* Row End(B) */
  MT9D113_write_cmos_sensor(0x990, 0x04BB); /* 1211 */
  MT9D113_write_cmos_sensor(0x98C, 0x2729); /* Column End(B) */
  MT9D113_write_cmos_sensor(0x990, 0x064B); /* 1611 */
  MT9D113_write_cmos_sensor(0x98C, 0x272B); /* Row Speed(B) */
  MT9D113_write_cmos_sensor(0x990, 0x0111); /* 273 */
  MT9D113_write_cmos_sensor(0x98C, 0x272F); /* sensor_fine_correction(B) */
  MT9D113_write_cmos_sensor(0x990, 0x003A); /* 58 */
  MT9D113_write_cmos_sensor(0x98C, 0x2731); /* sensor_fine_IT_min(B) */
  MT9D113_write_cmos_sensor(0x990, 0x00F6); /* 246 */
  MT9D113_write_cmos_sensor(0x98C, 0x2733); /* sensor_fine_IT_max_margin(B) */
  MT9D113_write_cmos_sensor(0x990, 0x008B); /* 139 */
  MT9D113_write_cmos_sensor(0x98C, 0x2739); /* Crop_X0(A) */
  MT9D113_write_cmos_sensor(0x990, 0x0000); /* 0 */
  MT9D113_write_cmos_sensor(0x98C, 0x273B); /* Crop_X1(A) */
  MT9D113_write_cmos_sensor(0x990, 0x031F); /* 799 */
  MT9D113_write_cmos_sensor(0x98C, 0x273D); /* Crop_Y0(A) */
  MT9D113_write_cmos_sensor(0x990, 0x0000); /* 0 */
  MT9D113_write_cmos_sensor(0x98C, 0x273F); /* Crop_Y1(A) */
  MT9D113_write_cmos_sensor(0x990, 0x0257); /* 599 */
  MT9D113_write_cmos_sensor(0x98C, 0x2747); /* Crop_X0(B) */
  MT9D113_write_cmos_sensor(0x990, 0x0000); /* 0 */
  MT9D113_write_cmos_sensor(0x98C, 0x2749); /* Crop_X1(B) */
  MT9D113_write_cmos_sensor(0x990, 0x063F); /* 1599 */
  MT9D113_write_cmos_sensor(0x98C, 0x274B); /* Crop_Y0(B) */
  MT9D113_write_cmos_sensor(0x990, 0x0000); /* 0 */
  MT9D113_write_cmos_sensor(0x98C, 0x274D); /* Crop_Y1(B) */
  MT9D113_write_cmos_sensor(0x990, 0x04AF); /* 1199 */
  /* no need config search_f1/f2_50/60, for they use in auto FD!!! */
  MT9D113_write_cmos_sensor(0x98C, 0xA404); /* FD Mode */
  MT9D113_write_cmos_sensor(0x990, 0x10  ); /* 16 */
  MT9D113_write_cmos_sensor(0x98C, 0xA40D); /* Stat_min */
  MT9D113_write_cmos_sensor(0x990, 0x02  ); /* 2 */
  MT9D113_write_cmos_sensor(0x98C, 0xA40E); /* Stat_max */
  MT9D113_write_cmos_sensor(0x990, 0x03  ); /* 3 */
  MT9D113_write_cmos_sensor(0x98C, 0xA410); /* Min_amplitude */
  MT9D113_write_cmos_sensor(0x990, 0x0A  ); /* 10 */
}


static void MT9D113_patch(void)
{
  const kal_uint8 rev = MT9D113_read_cmos_sensor(0x31FE)&0x000F;

  MT9D113_TRACE("MT9D113 Rev: %d", rev);

  switch (rev)
  {
#if (defined(MT9D113_REV2))
  case 0x02:
  {
    /* 2030SOC_REV02_PATCH02_REV9 */
    const static kal_uint16 patch_data[]=
    {
      0xF601,0x42C1,0x0226,0x11F6,0x0143,0xC102,0x260A,0xCC04,0x33BD,0xA365,0xBD04,0x3339,0xC6FF,0xF701,0x6439,0xFE02,
      0x5418,0xCE03,0x42CC,0x000B,0xBDC2,0xBFCC,0x04BF,0xFD03,0x4CCC,0x0342,0xFD02,0x545F,0x4FFD,0x025F,0xFE02,0xBD18,
      0xCE03,0x30C6,0x11BD,0xC2BF,0xCC05,0x03FD,0x033A,0xCC05,0xB5FD,0x033E,0xCC03,0x30FD,0x02BD,0xDE00,0x18CE,0x00C2,
      0xCC00,0x37BD,0xC2BF,0xCC06,0xAEDD,0xC4CC,0x06B4,0xDDD6,0xCC00,0xC2DD,0x00B6,0x02C3,0x810A,0x2407,0x1686,0x0A3D,
      0xF702,0xC3B6,0x02C4,0x810A,0x2407,0x1686,0x0A3D,0xF702,0xC4CC,0x02BD,0xFD03,0x4EFE,0x034E,0xCC4A,0x38ED,0x66C6,
      0x02F7,0x0164,0xC609,0xF701,0x6539,0x3C3C,0x34BD,0xCDCF,0x7D02,0x5626,0x05F6,0x025F,0x2003,0xF602,0x6030,0xE704,
      0xCC34,0x00BD,0xA55B,0xC43F,0x84FE,0x30ED,0x02E6,0x044F,0x0505,0x0505,0x0505,0xC4C0,0x8401,0xEA03,0xAA02,0xED02,
      0xCC34,0x00ED,0x00EC,0x02BD,0xA547,0x3838,0x3139,0x3CF6,0x02C3,0xF102,0xC423,0x08F0,0x02C4,0x30E7,0x0020,0x0330,
      0x6F00,0xF602,0xC3FB,0x02C4,0xE701,0x4FB3,0x02C8,0x2415,0xF602,0xC5F1,0x02C2,0x2308,0xF002,0xC2F7,0x02C5,0x2016,
      0x7F02,0xC520,0x11E6,0x004F,0xB302,0xC823,0x09F6,0x02C5,0xFB02,0xC2F7,0x02C5,0xF602,0xC5F1,0x02C1,0x2306,0xF602,
      0xC1F7,0x02C5,0x3839,0x3736,0x3C3C,0x3C3C,0x3C34,0x30EC,0x0FBD,0xA55B,0x30ED,0x0617,0x847F,0xA70A,0xEC06,0xC480,
      0x8401,0xED08,0x2709,0x8301,0x8026,0x0668,0x0A20,0x0264,0x0AEC,0x0BBD,0xA55B,0x30ED,0x06E6,0x0A4F,0xED02,0xEC06,
      0xED00,0xCC00,0x80BD,0xA409,0x30EC,0x02ED,0x046D,0x0427,0x05CC,0x00FF,0xED04,0xE605,0x308F,0xC300,0x0D8F,0x3539,
      0x308F,0xC3FF,0xF38F,0x35CC,0x3210,0xBDA5,0x5B30,0xED06,0xBDD6,0x54CC,0x3210,0x30ED,0x00EC,0x06BD,0xA547,0xCC30,
      0x5A30,0xED00,0xCC32,0xD4BD,0x055B,0x30E7,0x08CC,0x3056,0xED00,0xCC32,0xD6BD,0x055B,0x30E7,0x09CC,0x3058,0xED00,
      0xCC32,0xDABD,0x055B,0x30E7,0x0AE6,0x09E7,0x0B6F,0x0CE6,0x0C4F,0x05C3,0x33F6,0xED00,0xE60C,0x4FED,0x028F,0xC300,
      0x0830,0xE302,0x188F,0x3C18,0xE600,0x4F30,0xED00,0x3CF6,0x02F1,0x30ED,0x003C,0xF602,0xF230,0xED00,0xE612,0xC302,
      0xE98F,0x34E6,0x0030,0xE700,0xE613,0x4FC3,0x02ED,0x8FE6,0x00BD,0xA4EE,0x3838,0x3831,0x4FBD,0xA547,0x306C,0x0CE6,
      0x0CC1,0x0425,0xAACE,0x02BD,0x1F1F,0x8047,0xCC33,0xF4BD,0xA55B,0xC4BF,0x30ED,0x04FE,0x034E,0xFC02,0xD8A3,0x6625,
      0x0430,0x1C05,0x40CE,0x02BD,0x1F1F,0x401C,0xCC32,0x10BD,0xA55B,0xC4EF,0x30ED,0x061E,0x0540,0x031C,0x0710,0xCC32,
      0x10ED,0x00EC,0x06BD,0xA547,0xCC33,0xF430,0xED00,0xEC04,0xBDA5,0x4730,0xC60D,0x3A35,0x39BD,0xA705,0xBDA7,0x613C,
      0x3C3C,0x122F,0x0203,0x5F20,0x02C6,0x0130,0xE702,0xCC00,0x18BD,0xA55B,0x8580,0x2603,0x5F20,0x02C6,0x0130,0xE703,
      0xD604,0xF704,0x13CC,0x0016,0xBDA5,0x5BFD,0x040F,0xCC00,0x14BD,0xA55B,0xFD04,0x0DCC,0x001A,0xBDA5,0x5BFD,0x0411,
      0xDE00,0xEE24,0xAD00,0xCC00,0x1630,0xED00,0xDC30,0xBDA5,0x47CC,0x001A,0xBDA5,0x5B30,0xED04,0x1F04,0x020C,0xCC00,
      0x1AED,0x00EC,0x0484,0xFDBD,0xA547,0xDE00,0xEE36,0xC605,0xAD00,0xCC07,0xFFFD,0x1042,0xC603,0xF710,0x44F6,0x104D,
      0xC4F0,0xCA09,0xF710,0x4DCC,0x0020,0x30ED,0x00CC,0x0001,0xBDA5,0x47CC,0x0026,0x30ED,0x0034,0xBDA5,0x5BCA,0x0231,
      0xBDA5,0x47CC,0x0018,0x30ED,0x0034,0xBDA5,0x5B8A,0x4031,0xBDA5,0x4720,0x11C6,0xFFF7,0x1040,0xB610,0x40FE,0x0140,
      0xEE00,0xC620,0xAD00,0x306D,0x0227,0x0FCC,0x0018,0xBDA5,0x5BC4,0x0184,0x8083,0x0000,0x26DB,0x306D,0x0226,0x06D6,
      0x03C1,0x0126,0xD0CC,0x0016,0xED00,0xFC04,0x0FCA,0x20BD,0xA547,0xDE00,0xEE30,0xAD00,0xCC00,0x2630,0xED00,0x34BD,
      0xA55B,0xC4FD,0x31BD,0xA547,0xCC00,0x1A30,0xED00,0xEC04,0x8A08,0xBDA5,0x47CC,0x001A,0x30ED,0x00EC,0x04BD,0xA547,
      0xCC00,0x1630,0xED00,0xFC04,0x0FBD,0xA547,0xDE00,0xEE2E,0xAD00,0x306D,0x0226,0x06DE,0x00EE,0x2CAD,0x00F6,0x104D,
      0xC4F0,0xCA0A,0xF710,0x4D38,0x3838
    };
    MT9D113_burst_write_XDMA(0x0415, patch_data, sizeof(patch_data) / sizeof(patch_data[0]));
    MT9D113_write_XDMA(0x87FF, 0x0039);
    break;
  }
#endif
#if (defined(MT9D113_REV3))
  case 0x03:
  {
    /* K25A_REV03_PATCH01_REV2 */
    const static kal_uint16 patch_data[]=
    {
      0xF601,0x42C1,0x0326,0x11F6,0x0143,0xC104,0x260A,0xCC04,0x33BD,0xA362,0xBD04,0x3339,0xC6FF,0xF701,0x6439,0xFE02,
      0xBD18,0xCE03,0x25CC,0x0011,0xBDC2,0xB8CC,0x0470,0xFD03,0x33CC,0x0325,0xFD02,0xBDDE,0x0018,0xCE03,0x37CC,0x0037,
      0xBDC2,0xB8CC,0x0497,0xFD03,0x5BCC,0x0337,0xDD00,0xC601,0xF701,0x645C,0xF701,0x657F,0x0166,0x39BD,0xD661,0xF602,
      0xF4C1,0x0126,0x0BFE,0x02BD,0xEE10,0xFC02,0xF5AD,0x0039,0xF602,0xF4C1,0x0226,0x0AFE,0x02BD,0xEE10,0xFC02,0xF7AD,
      0x0039,0x3CBD,0xB059,0xCC00,0x28BD,0xA558,0x8300,0x0027,0x0BCC,0x0026,0x30ED,0x00C6,0x03BD,0xA544,0x3839
    };
    MT9D113_burst_write_XDMA(0x0415, patch_data, sizeof(patch_data) / sizeof(patch_data[0]));
    break;
  }
#endif
  default:
    ASSERT(0);
  }
  MT9D113_write_XDMA(0x2006, 0x0415); /* MON_ARG1 */
  MT9D113_write_XDMA(0xA005, 0x01); /* MON_CMD */
  mdelay(5); /* wait for the patch to complete initialization */
}


static void MT9D113_initial_setting(void)
{
	//MT9D113_write_cmos_sensor(0x3012, 0x80); /* [7]: software reset */
	MT9D113_sensor.initial = MT9D113_sensor.first_pv = KAL_TRUE;
	/* Reset */
	  MT9D113_sensor.pclk = 77000000; /* 26MHz CLKIN, 77 MHz PLL out */

	MT9D113_write_cmos_sensor(0x001A, 0x0051);
	mdelay(5);
	MT9D113_write_cmos_sensor(0x001A, 0x0050);
	mdelay(5);

	/* PLL VCO = MCLK * 2 * M / (N + 1), PCLK = VCO / (P1 + 1) */
	MT9D113_write_cmos_sensor(0x0014, 0x21F9); /* PLL control: BYPASS PLL */
	MT9D113_write_cmos_sensor(0x0010, 0x0C9A); /* PLL Dividers [7~0]: m, [13~8]: n */
	MT9D113_write_cmos_sensor(0x0012, 0x1FF7); /* PLL P Dividers [3~0]: p1, [7~4]: p2, [11~8]: p3 */

	MT9D113_write_cmos_sensor(0x0014, 0x21FB);  /* PLL control: PLL_ENABLE on = 8699 */
	MT9D113_write_cmos_sensor(0x0014, 0x20FB);  /* PLL control: SEL_LOCK_DET on = 8443 */
	mdelay(5); /* Allow PLL to lock */
	MT9D113_write_cmos_sensor(0x0014, 0x20FA);  /* PLL control: PLL_BYPASS off = 8442 */
	MT9D113_write_cmos_sensor(0x321C, 0x0003);  /* By Pass TxFIFO = 3 */
	MT9D113_write_cmos_sensor(0x001E, 0x0403); /* PAD_SLEW */

	MT9D113_write_cmos_sensor(0x0018, 0x402D); /* MCU Powerup stop enable */
	MT9D113_write_cmos_sensor(0x0018, 0x402C); /* GO */
	mdelay(20); /* wait for R20B to come out of standby */

	MT9D113_config_context();

	MT9D113_sensor.pv_frame_height = MT9D113_sensor.pv_line_length = 0; /* force config preview dummy */
	MT9D113_set_pv_dummy(0, 0);

	MT9D113_sensor.cap_frame_height = MT9D113_sensor.cap_line_length = 0; /* force config capture dummy */
	MT9D113_set_cap_dummy(0, 0);

	/* Errata for Rev2 */
	MT9D113_write_cmos_sensor(0x3084, 0x240C);
	MT9D113_write_cmos_sensor(0x3092, 0x0A4C);
	MT9D113_write_cmos_sensor(0x3094, 0x4C4C);
	MT9D113_write_cmos_sensor(0x3096, 0x4C54);

	MT9D113_LSC_setting();
	MT9D113_gamma_setting();
	MT9D113_low_light();
	MT9D113_awb_ccms();

	MT9D113_write_cmos_sensor(0x3240, 0xC807); /* LUM_LIMITS_WB_STATS: change AWB */
	MT9D113_write_XDMA(0xA768, 0x05); /* highlight capture look like purple issue, default is 4 */
	MT9D113_write_XDMA(0xA115, 0x02); /* seq_cap_mode */
	MT9D113_write_XDMA(0xA117, 0x00); /* seq_preview_0_ae */
	MT9D113_write_XDMA(0xA129, 0x00); /* seq_preview_3_ae */
	MT9D113_write_XDMA(0xA209, 0x20); /* AE */
	MT9D113_write_XDMA(0xA20A, 0x03); /* AE */
	MT9D113_write_XDMA(0xA109, 0x08); /* seq_ae_fast_buff */
	MT9D113_write_XDMA(0xA10A, 0x01); /* seq_ae_fast_step */
	MT9D113_write_XDMA(0xA20E, MT9D113_MAX_ANALOG_GAIN); /* ae_max_virt_gain */

	/* default setting */
	MT9D113_sensor.video_mode = KAL_FALSE;
	MT9D113_sensor.normal_fps = MT9D113_FPS(10);
	MT9D113_sensor.night_fps = MT9D113_FPS(5);
	MT9D113_sensor.Fps = MT9D113_FPS(10);

	MT9D113_sensor.mirror = !IMAGE_NORMAL; /* force config mirror */
	MT9D113_set_mirror(IMAGE_NORMAL);

	MT9D113_sensor.banding = !AE_FLICKER_MODE_50HZ; /* force config banding */
	MT9D113YUVSensorSetting(FID_AE_FLICKER, AE_FLICKER_MODE_50HZ);

	MT9D113_sensor.effect = !MEFFECT_OFF; /* force config effect */
	MT9D113YUVSensorSetting(FID_COLOR_EFFECT, MEFFECT_OFF);

	MT9D113_sensor.exposure = !AE_EV_COMP_00; /* force config exposure */
	MT9D113YUVSensorSetting(FID_AE_EV, AE_EV_COMP_00);

	MT9D113_sensor.wb = !AWB_MODE_AUTO; /* force config wb */
	MT9D113YUVSensorSetting(FID_AWB_MODE, AWB_MODE_AUTO);

	MT9D113_sensor.night_mode = !KAL_FALSE; /* force config night mode */
	MT9D113YUVSensorSetting(FID_SCENE_MODE, SCENE_MODE_OFF);

	MT9D113_sensor.enable_3a = !KAL_TRUE; /* force config 3a */
	MT9D113_set_param_3a_enable(KAL_TRUE);

	MT9D113_patch();

#ifdef __MT9D113_TEST_PATTERN__
	/* 00: disabled 01: solid white 02: grey ramp 03: color bar ramp 04: solid white (color bars) 05: noise */
	MT9D113_write_XDMA(0xA766, 0x03);
#endif
	MT9D113_write_cmos_sensor(0x0018, 0x0028); /* Continue */
	mdelay(20); /* wait for sequencer to enter preview state */
	MT9D113_sensor.initial = KAL_FALSE;
	MT9D113_set_cmd(6); /* refresh mode */
	MT9D113_set_cmd(5); /* refresh */

	MT9D113_sensor.pv_mode = KAL_TRUE;
}

static kal_uint16 MT9D113_gain2shutter(kal_bool enable)
{
  static kal_uint16 gain_r, gain_gr, gain_gb, gain_b;
  kal_uint8 mul = 2; /* multiple of gain */
  
  if (enable)
  {
    gain_gr = MT9D113_read_cmos_sensor(0x32D6);
    /* convert digital gain(isp gain. sensor core digital gain no use) */
    while (gain_gr >= mul * 0x80) mul++;
    if (--mul > 1)
    {
      gain_r = MT9D113_read_cmos_sensor(0x32D4);
      gain_gb = MT9D113_read_cmos_sensor(0x32D8);
      gain_b = MT9D113_read_cmos_sensor(0x32DA);
      MT9D113_write_cmos_sensor(0x32D4, MT9D113_half_adjust(gain_r, mul));
      MT9D113_write_cmos_sensor(0x32D6, MT9D113_half_adjust(gain_gr, mul));
      MT9D113_write_cmos_sensor(0x32D8, MT9D113_half_adjust(gain_gr, mul));
      MT9D113_write_cmos_sensor(0x32DA, MT9D113_half_adjust(gain_b * gain_gr, mul * gain_gb));
      
      return mul;
    }
    gain_gr = 0;
  }
  if (gain_r)  MT9D113_write_cmos_sensor(0x32D4, gain_r);
  if (gain_gr) MT9D113_write_cmos_sensor(0x32D6, gain_gr);
  if (gain_gb) MT9D113_write_cmos_sensor(0x32D8, gain_gr);
  if (gain_b)  MT9D113_write_cmos_sensor(0x32DA, gain_b);
  gain_r = gain_gr = gain_gb = gain_b = 0;
  
  return 1;
}

static kal_uint16 MT9D113_power_on(void)
{
  kal_uint16 sensor_id;
  
  // 26MHz CLKIN, 52 MHz PLL out
  MT9D113_sensor.pclk = 52000000;
  MT9D113_sensor.write_id = MT9D113_SLV1_WRITE_ID;
  MT9D113_sensor.read_id = MT9D113_SLV1_READ_ID;
  sensor_id = MT9D113_read_cmos_sensor(0x0000);
  
  MT9D113_TRACE("MT9D113 SENSOR ID: %x, I2C Addr=%x", sensor_id, MT9D113_sensor.write_id);
  if (sensor_id != MT9D113_SENSOR_ID) /* READ SENSOR ID */
  {
    MT9D113_sensor.write_id = MT9D113_SLV2_WRITE_ID;
    MT9D113_sensor.read_id = MT9D113_SLV2_READ_ID;

    sensor_id = MT9D113_read_cmos_sensor(0x0000);
    if (sensor_id != MT9D113_SENSOR_ID)
    {
      MT9D113_TRACE("MT9D113 SENSOR ID: %x, I2C Addr=%x", sensor_id, MT9D113_sensor.write_id);
    }
  }
  return sensor_id;
}

void MT9D113_NightMode(kal_bool bEnable)
{    
	kal_uint8 y_rgb_offset, awb_gain_max_r;
	kal_uint16 limit_of_digital_gain;

	if (MT9D113_sensor.night_mode == bEnable )
		return;
	
	MT9D113_sensor.night_mode = bEnable;
	if (MT9D113_sensor.night_mode)
	{
		MT9D113_write_XDMA(0xAB37, 0x02); /* use GAMMA TABLE B */
		y_rgb_offset = 20;
		awb_gain_max_r = 0xA0;
		limit_of_digital_gain = 0x0100;
	}
	else
	{
		MT9D113_write_XDMA(0xAB37, 0x01); /* use GAMMA TABLE A */
		y_rgb_offset = 0;
		awb_gain_max_r = 0xC8;
		limit_of_digital_gain = 0x00EE;
	}
	MT9D113_write_XDMA(0x2212, limit_of_digital_gain);
	MT9D113_write_XDMA(0xA34B, awb_gain_max_r);
	MT9D113_write_XDMA(0xA75D, y_rgb_offset); /* mode_y_rgb_offset_a */
	MT9D113_write_XDMA(0xA75E, y_rgb_offset); /* mode_y_rgb_offset_b */

	MT9D113_cal_fps(); /* need cal new fps */
}   /* MT9D113_NightMode */

UINT32 MT9D113GetSensorID(UINT32 *sensor_id) 
{
    *sensor_id = MT9D113_power_on();
	
    if (*sensor_id != MT9D113_SENSOR_ID)
	{
		MT9D113_TRACE("Read Sensor ID Fail = 0x%04x\n", *sensor_id); 
        *sensor_id = 0xFFFFFFFF; 
		
        return ERROR_SENSOR_CONNECT_FAIL;
    }
	
    return ERROR_NONE;
}


UINT32 MT9D113Open(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

	mdelay(20);
	
	if (MT9D113_SENSOR_ID != MT9D113_power_on())
		return ERROR_SENSOR_CONNECT_FAIL;

	MT9D113_initial_setting();

	return ERROR_NONE;
}	/* MT9D113Open() */

UINT32 MT9D113Close(void)
{
	return ERROR_NONE;
}	/* MT9D113Close() */

UINT32 MT9D113Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 dummy_pixel;

	MT9D113_sensor.video_mode = KAL_FALSE;
    MT9D113_sensor.normal_fps = MT9D113_FRAME_RATE_UNIT * 10;
    MT9D113_sensor.night_fps = MT9D113_FRAME_RATE_UNIT * 5;
    dummy_pixel = 0;
	if (!MT9D113_sensor.pv_mode)
	{
		MT9D113_write_cmos_sensor(0x3012, MT9D113_sensor.shutter); /* shutter */
		MT9D113_gain2shutter(KAL_FALSE);
	}
	
	// PCLK Inverse, If need inverse the pclk, plz unmark bellow code.
	//MT9D113_write_cmos_sensor(0x098c, 0x2755); // Context A
	//MT9D113_write_cmos_sensor(0x0990, 0x0200);
	//MT9D113_write_cmos_sensor(0x098c, 0x2757);	// Context B
	//MT9D113_write_cmos_sensor(0x0990, 0x0200);
	//MT9D113_write_cmos_sensor(0x098c, 0xA103);
	//MT9D113_write_cmos_sensor(0x0990, 0x0006);

	MT9D113_set_pv_dummy(dummy_pixel, 0);
	MT9D113_set_cmd(1); /* seq_cmd: goto preview mode */

    image_window->GrabStartX= MT9D113_PV_X_START;
    image_window->GrabStartY = MT9D113_PV_Y_START;
    image_window->ExposureWindowWidth = MT9D113_IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight = MT9D113_IMAGE_SENSOR_PV_HEIGHT;

    return TRUE; 
}	/* MT9D113Preview() */

UINT32 MT9D113Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 dummy_pixel;
	kal_uint16 shutter;
	kal_uint8 mul = 1; /* multiple of gain to shutter */
  
	if (image_window->ImageTargetWidth <= MT9D113_IMAGE_SENSOR_PV_WIDTH_DRV &&
	image_window->ImageTargetHeight <= MT9D113_IMAGE_SENSOR_PV_HEIGHT_DRV)
	{
		dummy_pixel = 0;
		MT9D113_set_pv_dummy(dummy_pixel, 0);
		image_window->GrabStartX = MT9D113_PV_X_START;
		image_window->GrabStartY = MT9D113_PV_Y_START;
		image_window->ExposureWindowWidth = MT9D113_IMAGE_SENSOR_PV_WIDTH;
		image_window->ExposureWindowHeight = MT9D113_IMAGE_SENSOR_PV_HEIGHT;
	}
	else
	{
		if (image_window->ImageTargetWidth > MT9D113_IMAGE_SENSOR_FULL_WIDTH_DRV &&
		image_window->ImageTargetHeight > MT9D113_IMAGE_SENSOR_FULL_HEIGHT_DRV &&
		MT9D113_zoom_factor >=  2)
		{
			dummy_pixel = 0x6B0;
		}
		else if (MT9D113_zoom_factor >=  3)
		{
			dummy_pixel = 0x110;
		}
		else
		{
			dummy_pixel = 0;
		}
		
		MT9D113_sensor.shutter = MT9D113_read_cmos_sensor(0x3012); /* back up shutter */
		MT9D113_set_cap_dummy(dummy_pixel, 0);
		MT9D113_set_cmd(2); /* seq_cmd: goto capture mode */
		shutter = MT9D113_half_adjust(MT9D113_sensor.pv_line_length * MT9D113_sensor.shutter, MT9D113_sensor.cap_line_length);
		mul = MT9D113_gain2shutter(KAL_TRUE); /* transfer gain to shutter */
		
		if (!shutter)
		{
			shutter = 1; /* avoid shutter 0 */
		}
		MT9D113_write_cmos_sensor(0x3012, shutter * mul);
		
		image_window->GrabStartX = MT9D113_FULL_X_START;
		image_window->GrabStartY = MT9D113_FULL_Y_START;
		image_window->ExposureWindowWidth = MT9D113_IMAGE_SENSOR_FULL_WIDTH;
		image_window->ExposureWindowHeight = MT9D113_IMAGE_SENSOR_FULL_HEIGHT;
	}

	return ERROR_NONE;
}	/* MT9D113Capture() */

UINT32 MT9D113GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth =		MT9D113_IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorFullHeight =		MT9D113_IMAGE_SENSOR_FULL_HEIGHT;
	pSensorResolution->SensorPreviewWidth =		MT9D113_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight =	MT9D113_IMAGE_SENSOR_PV_HEIGHT;

	return ERROR_NONE;
}	/* MT9D113GetResolution() */

UINT32 MT9D113GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_PARALLEL;
	
	pSensorInfo->SensorPreviewResolutionX =		MT9D113_IMAGE_SENSOR_PV_HACTIVE;
	pSensorInfo->SensorPreviewResolutionY =		MT9D113_IMAGE_SENSOR_PV_VACTIVE;
	pSensorInfo->SensorFullResolutionX =		MT9D113_IMAGE_SENSOR_FULL_HACTIVE;
	pSensorInfo->SensorFullResolutionY =		MT9D113_IMAGE_SENSOR_FULL_VACTIVE;

	pSensorInfo->SensorCameraPreviewFrameRate =		30;
	pSensorInfo->SensorVideoFrameRate =				30;
	pSensorInfo->SensorStillCaptureFrameRate =		10;
	pSensorInfo->SensorWebCamCaptureFrameRate =		15;
	
	pSensorInfo->SensorResetActiveHigh =			FALSE;
	pSensorInfo->SensorResetDelayCount =			1;
	pSensorInfo->SensorOutputDataFormat =			SENSOR_OUTPUT_FORMAT_VYUY;
	
	pSensorInfo->SensorClockPolarity =				SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorClockFallingPolarity =		SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = 				SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = 				SENSOR_CLOCK_POLARITY_HIGH; 
	
	pSensorInfo->SensorInterruptDelayLines = 		1;

	pSensorInfo->CaptureDelayFrame = 				2; 
	pSensorInfo->PreviewDelayFrame = 				5; 
	pSensorInfo->VideoDelayFrame = 					5; 		
	
	pSensorInfo->SensorMasterClockSwitch = 			0; 
    pSensorInfo->SensorDrivingCurrent = 			ISP_DRIVING_4MA; 		
   
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
		default:
			pSensorInfo->SensorClockFreq =				26;
			pSensorInfo->SensorClockDividCount =		3;
			pSensorInfo->SensorClockRisingCount = 		0;
			pSensorInfo->SensorClockFallingCount = 		2;
			pSensorInfo->SensorPixelClockCount = 		3;
			pSensorInfo->SensorDataLatchCount = 		2;

			pSensorInfo->SensorGrabStartX = 			3; 
			pSensorInfo->SensorGrabStartY = 			1;       
		break;
	}

	memcpy(pSensorConfigData, &MT9D113SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

	return ERROR_NONE;
}	/* MT9D113GetInfo() */


UINT32 MT9D113Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			MT9D113Preview(pImageWindow, pSensorConfigData);
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			MT9D113Capture(pImageWindow, pSensorConfigData);
		break;
		default:
		    break; 
	}
	return TRUE;
}	/* MT9D113Control() */

BOOL MT9D113_set_param_wb(UINT16 para)
{
	kal_uint8 awb_ccm_position;
	  
	if (MT9D113_sensor.wb == para) return KAL_TRUE;
	  MT9D113_sensor.wb = para;

    switch (MT9D113_sensor.wb)
    {
        case AWB_MODE_AUTO:   // enable AWB
			MT9D113_write_XDMA(0xA355, 0x02);
            break;

		case AWB_MODE_CLOUDY_DAYLIGHT:
			awb_ccm_position = 0x6F;
			break;
			
        case AWB_MODE_DAYLIGHT:
            awb_ccm_position = 0x7F;
            break;
        
        case AWB_MODE_INCANDESCENT:
			awb_ccm_position = 0x10;
            break;
        
        case AWB_MODE_FLUORESCENT:
			awb_ccm_position = 0x79;
            break;
        
        case AWB_MODE_TUNGSTEN:
			awb_ccm_position = 0x00;
            break;
        
        default:
            return FALSE;
    }
	MT9D113_write_XDMA(0xA355, 0x22); /* awb_mode */
    MT9D113_write_XDMA(0xA353, awb_ccm_position); /* awb_ccm_position */
    return TRUE;
} /* MT9D113_set_param_wb */

BOOL MT9D113_set_param_effect(UINT16 para)
{
	kal_uint8 spec_effects;
	  
	  if (MT9D113_sensor.effect == para) return KAL_TRUE;
	  MT9D113_sensor.effect = para;

    switch (MT9D113_sensor.effect)
    {
        case MEFFECT_OFF:
			spec_effects = 0;
            break;

		case MEFFECT_MONO:
		    spec_effects = 1;
            break;
			
        case MEFFECT_SEPIA:
			spec_effects = 2;
			MT9D113_write_XDMA(0x2763, 0xB023); /* mode_common_mode_settings_fx_sepia_settings */
            break;

        case MEFFECT_SEPIAGREEN:
			spec_effects = 2;
			MT9D113_write_XDMA(0x2763, 0xB0CD);
            break;

        case MEFFECT_SEPIABLUE:
	       spec_effects = 2;
		    MT9D113_write_XDMA(0x2763, 0x25F0);
            break;
        
        case MEFFECT_NEGATIVE:
			spec_effects = 3;
            break;
      
        default:
            return FALSE;
    }
	MT9D113_write_XDMA(0x2759, 0x6440|spec_effects); /* preview spec_effects */
	MT9D113_write_XDMA(0x275B, 0x6440|spec_effects); /* snapshot/video spec_effects */
	MT9D113_set_cmd(5); /* refresh */
	  
    return TRUE;
} /* MT9D113_set_param_effect */

BOOL MT9D113_set_param_banding(UINT16 para)
{
	if (MT9D113_sensor.banding == para) return KAL_TRUE;
	MT9D113_sensor.banding = para;
	
	switch (MT9D113_sensor.banding)
	{
		case AE_FLICKER_MODE_50HZ:
		MT9D113_write_XDMA(0xA404, 0xC0); /* fd_mode */
		break;
		
		case AE_FLICKER_MODE_60HZ:
		MT9D113_write_XDMA(0xA404, 0x80); /* fd_mode */
		break;
		
		default:
		return FALSE;
	}
	MT9D113_set_cmd(6); /* refresh mode */
	MT9D113_cal_fps(); /* need cal new fps */

	return TRUE;
} /* MT9D113_set_param_banding */

BOOL MT9D113_set_param_exposure(UINT16 para)
{
	kal_uint8 ae_base_target;
	
	if (MT9D113_sensor.exposure == para) return KAL_TRUE;
	  MT9D113_sensor.exposure = para;
    switch (MT9D113_sensor.exposure)
    {
        case AE_EV_COMP_n13:    // -4 EV            
			ae_base_target = MT9D113_AE_TARGET_ZERO - 40;
            break;
        
        case AE_EV_COMP_n10:    // -3 EV
			ae_base_target = MT9D113_AE_TARGET_ZERO - 30;
            break;
        
        case AE_EV_COMP_n07:    // -2 EV
			ae_base_target = MT9D113_AE_TARGET_ZERO - 20;
            break;
        
        case AE_EV_COMP_n03:    // -1 EV
			ae_base_target = MT9D113_AE_TARGET_ZERO - 10;
            break;
        
        case AE_EV_COMP_00:   // +0 EV
			ae_base_target = MT9D113_AE_TARGET_ZERO;
            break;
        
        case AE_EV_COMP_03:    // +1 EV
			ae_base_target = MT9D113_AE_TARGET_ZERO + 10;
            break;
        
        case AE_EV_COMP_07:    // +2 EV
			ae_base_target = MT9D113_AE_TARGET_ZERO + 20;
            break;
        
        case AE_EV_COMP_10:    // +3 EV
			ae_base_target = MT9D113_AE_TARGET_ZERO + 30;
            break;
        
        case AE_EV_COMP_13:    // +4 EV
			ae_base_target = MT9D113_AE_TARGET_ZERO + 40;
            break;
        
        default:
            return FALSE;    
    }
	MT9D113_write_XDMA(0xA24F, ae_base_target); /* ae_base_target */

    return TRUE;	
} /* MT9D113_set_param_exposure */



UINT32 MT9D113YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
	switch (iCmd) {
	case FID_SCENE_MODE:	    
	    if (iPara == SCENE_MODE_OFF)
	    {
	        MT9D113_NightMode(FALSE); 
	    }
	    else if (iPara == SCENE_MODE_NIGHTSCENE)
	    {
			MT9D113_NightMode(TRUE); 
	    }	    
	break; 	    
	case FID_AWB_MODE:
		MT9D113_set_param_wb(iPara);
	break;
	case FID_COLOR_EFFECT:
		MT9D113_set_param_effect(iPara);
	break;
	case FID_AE_EV:
		MT9D113_set_param_exposure(iPara);
	break;
	case FID_AE_FLICKER:
		MT9D113_set_param_banding(iPara);
	break;
	case FID_ZOOM_FACTOR:
		MT9D113_zoom_factor = iPara;
	break;
	default:
	break;
	}
	
	return TRUE;
}   /* MT9D113YUVSensorSetting */

UINT32 MT9D113YUVSetVideoMode(UINT16 u2FrameRate)
{
	MT9D113_sensor.video_mode = TRUE;
	MT9D113_sensor.normal_fps = MT9D113_FRAME_RATE_UNIT * 30;
    MT9D113_sensor.night_fps = MT9D113_FRAME_RATE_UNIT * 15;
	
	if (u2FrameRate == 30)
    {
		MT9D113_sensor.video_frame = FALSE;
    }
    else if (u2FrameRate == 15)       
    {
		MT9D113_sensor.video_frame = TRUE;
    }
    else 
    {
        MT9D113_TRACE("Wrong frame rate setting \n");
    }   
	
    MT9D113_cal_fps();
	
    return TRUE;
}

UINT32 MT9D113FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 u2Temp = 0; 
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=MT9D113_IMAGE_SENSOR_FULL_HACTIVE;
			*pFeatureReturnPara16=MT9D113_IMAGE_SENSOR_FULL_VACTIVE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++=MT9D113_sensor.pv_frame_height;//+MT9D113_PV_dummy_pixels;
			*pFeatureReturnPara16=MT9D113_sensor.pv_line_length;//+MT9D113_PV_dummy_lines;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			MT9D113_NightMode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:
			break; 
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			MT9D113_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = MT9D113_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &MT9D113SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			MT9D113YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			MT9D113YUVSetVideoMode(*pFeatureData16);
        break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			MT9D113GetSensorID(pFeatureData32); 
		break; 	
		default:
			break;			
	}
	return ERROR_NONE;
}	/* MT9D113FeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncMT9D113=
{
	MT9D113Open,
	MT9D113GetInfo,
	MT9D113GetResolution,
	MT9D113FeatureControl,
	MT9D113Control,
	MT9D113Close
};

UINT32 MT9D113_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncMT9D113;

	return ERROR_NONE;
}	/* MT9D113_YUV_SensorInit() */


