
#ifndef  _qemu_android_h
#define  _qemu_android_h

#define  CONFIG_SHAPER  1

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/** in vl.c */

/* emulated network up/down speeds, expressed in bits/seconds */
extern double   qemu_net_upload_speed;
extern double   qemu_net_download_speed;

/* emulated network min-max latency, expressed in ms */
extern int      qemu_net_min_latency;
extern int      qemu_net_max_latency;

/* global flag, when true, network is disabled */
extern int      qemu_net_disable;

/* list of supported network speed names and values in bits/seconds */
typedef struct {
    const char*  name;
    const char*  display;
    int          upload;
    int          download;
} NetworkSpeed;

extern const NetworkSpeed   android_netspeeds[];

/* list of supported network latency names and min-max values in ms */
typedef struct {
    const char*  name;
    const char*  display;
    int          min_ms;
    int          max_ms;
} NetworkLatency;

extern const NetworkLatency  android_netdelays[];

/* default network settings for emulator */
#define  DEFAULT_NETSPEED  "full"
#define  DEFAULT_NETDELAY  "none"

extern void  qemu_polling_enable(void);
extern void  qemu_polling_disable(void);

/**in hw/goldfish_fb.c */

/* framebuffer dimensions in pixels, note these can change dynamically */
extern int  android_framebuffer_w;
extern int  android_framebuffer_h;
/* framebuffer dimensions in mm */
extern int  android_framebuffer_phys_w;
extern int  android_framebuffer_phys_h;

/* framebuffer rotation, relative to device */
typedef enum {
    ANDROID_ROTATION_0 = 0,
    ANDROID_ROTATION_90,
    ANDROID_ROTATION_180,
    ANDROID_ROTATION_270
} AndroidRotation;

extern AndroidRotation  android_framebuffer_rotation;

/**  in android_main.c */

extern int   android_base_port;

extern int   android_parse_network_speed(const char*  speed);

extern int   android_parse_network_latency(const char*  delay);

extern void  android_emulation_setup( void );
extern void  android_emulation_teardown( void );

#endif /* _qemu_android_h */
