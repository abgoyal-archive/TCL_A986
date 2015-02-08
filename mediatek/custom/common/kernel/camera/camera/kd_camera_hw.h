
#ifndef _KD_CAMERA_HW_H_
#define _KD_CAMERA_HW_H_
 
#if defined(MT6516)

#include <mach/mt6516_pll.h>
#include <mach/mt6516_gpio.h>
#include <mach/mt6516_gpt_sw.h>

//FIXME, should defined in DCT tool 
//
#ifndef GPIO_CAMERA_CMRST1_PIN
#define GPIO_CAMERA_CMRST1_PIN 0xFF
#endif
//
#ifndef GPIO_CAMERA_CMRST1_PIN_M_GPIO
#define GPIO_CAMERA_CMRST1_PIN_M_GPIO GPIO_MODE_00
#endif
//
#ifndef GPIO_CAMERA_CMPDN1_PIN
#define GPIO_CAMERA_CMPDN1_PIN 0xFF
#endif
//
#ifndef GPIO_CAMERA_CMPDN1_PIN_M_GPIO
#define GPIO_CAMERA_CMPDN1_PIN_M_GPIO GPIO_MODE_00
#endif
//
#define CAMERA_POWER_VCAM_A MT6516_POWER_VCAM_A
#define CAMERA_POWER_VCAM_D MT6516_POWER_VCAM_D
#define IMG_SENSOR_I2C_GROUP_ID 0

#elif defined(MT6573)

#include <mach/mt6573_pll.h>
#include <mach/mt6573_gpio.h>
#include <mach/mt6573_gpt.h>

//Power 
#define CAMERA_POWER_VCAM_A MT65XX_POWER_LDO_VCAMA
#define CAMERA_POWER_VCAM_D MT65XX_POWER_LDO_VCAMD
#define CAMERA_POWER_VCAM_A2 MT65XX_POWER_LDO_VCAMA2
#define CAMERA_POWER_VCAM_D2 MT65XX_POWER_LDO_VCAMD2

//FIXME, should defined in DCT tool 
//
#ifndef GPIO_CAMERA_LDO_EN_PIN 
#define GPIO_CAMERA_LDO_EN_PIN GPIO94
#endif 
//
#ifndef GPIO_CAMERA_CMRST_PIN 
#define GPIO_CAMERA_CMRST_PIN GPIO78
#endif 
//
#ifndef GPIO_CAMERA_CMRST_PIN_M_GPIO
#define GPIO_CAMERA_CMRST_PIN_M_GPIO GPIO_MODE_00
#endif 
//
#ifndef GPIO_CAMERA_CMPDN_PIN 
#define GPIO_CAMERA_CMPDN_PIN GPIO79
#endif 
//
#ifndef GPIO_CAMERA_LDO_EN_PIN_M_GPIO
#define GPIO_CAMERA_LDO_EN_PIN_M_GPIO GPIO_MODE_00
#endif 
//
#ifndef GPIO_CAMERA_CMPDN_PIN_M_GPIO
#define GPIO_CAMERA_CMPDN_PIN_M_GPIO  GPIO_MODE_00 
#endif 
//
#ifndef GPIO_CAMERA_CMRST1_PIN
#define GPIO_CAMERA_CMRST1_PIN GPIO97
#endif
//
#ifndef GPIO_CAMERA_CMRST1_PIN_M_GPIO
#define GPIO_CAMERA_CMRST1_PIN_M_GPIO GPIO_MODE_00
#endif
//
#ifndef GPIO_CAMERA_CMPDN1_PIN
#define GPIO_CAMERA_CMPDN1_PIN GPIO96
#endif
//
#ifndef GPIO_CAMERA_CMPDN1_PIN_M_GPIO
#define GPIO_CAMERA_CMPDN1_PIN_M_GPIO GPIO_MODE_00
#endif
//i2c id for sensor device, MT6573, the I2C is attached on 1
#define IMG_SENSOR_I2C_GROUP_ID 1
#else 
#error Error!!!, Forget to define the MACRO for Sensor Power and I2C control
#endif 
#endif 
