


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#ifdef MT6573
#include <mach/mt6573_typedefs.h>
#else
#include <mach/mt6516_typedefs.h>
#endif

#ifndef _YUSU_ANDROID_SPEAKER_H_
#define _YUSU_ANDROID_SPEAKER_H_

enum SPEAKER_CHANNEL
{
      Channel_None = 0 ,
      Channel_Right,
      Channel_Left,
      Channel_Stereo
};

void Sound_Speaker_Turnon(int channel);
void Sound_Speaker_Turnoff(int channel);
void Sound_Speaker_SetVolLevel(int level);
bool Speaker_Init(void);


void Sound_Headset_Turnon(void);
void Sound_Headset_Turnoff(void);

kal_int32 Sound_ExtFunction(const char* name, void* param, int param_size);

static int Speaker_Volume=0;

#endif


