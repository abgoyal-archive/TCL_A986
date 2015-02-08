
#ifndef _android_sensors_h
#define _android_sensors_h

#include "qemu-common.h"

/* initialize sensor emulation */
extern void  android_hw_sensors_init( void );

#define  SENSORS_LIST  \
    SENSOR_(ACCELERATION,"acceleration") \
    SENSOR_(MAGNETIC_FIELD,"magnetic-field") \
    SENSOR_(ORIENTATION,"orientation") \
    SENSOR_(TEMPERATURE,"temperature") \

typedef enum {
#define  SENSOR_(x,y)  ANDROID_SENSOR_##x,
    SENSORS_LIST
#undef   SENSOR_
    MAX_SENSORS  /* do not remove */
} AndroidSensor;

extern void  android_hw_sensor_enable( AndroidSensor  sensor );

/* COARSE ORIENTATION VALUES */
typedef enum {
    ANDROID_COARSE_PORTRAIT,
    ANDROID_COARSE_LANDSCAPE
} AndroidCoarseOrientation;

/* change the coarse orientation value */
extern void  android_sensors_set_coarse_orientation( AndroidCoarseOrientation  orient );

// +MTK03764_2011_03_09
// Add functions that can change sensor values at run-time.
extern void android_hw_set_acceleration(float, float, float);
extern void android_hw_set_magnetic_field(float, float, float);
extern void android_hw_set_orientation(float, float, float);
extern void android_hw_set_temperature(float);
// -MTK03764_2011_03_09
#endif /* _android_gps_h */
