


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <mach/mt6573_gpio.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <mach/mt6573_typedefs.h>
#include <linux/pmic6326_sw.h>
#include <linux/delay.h>
#include <mach/mt6573_pll.h>
#include "yusu_android_speaker.h"

//#define CONFIG_DEBUG_MSG
#ifdef CONFIG_DEBUG_MSG
#define PRINTK(format, args...) printk( KERN_EMERG format,##args )
#else
#define PRINTK(format, args...)
#endif


#define HEARING_AID_KERNEL //Add
#define HEARING_AID_KERNEL_DEBUG//Add
#ifdef HEARING_AID_KERNEL_DEBUG
#define PRINTT3(format, args...) printk( KERN_EMERG format,##args )
#else
#define PRINTT3(format, args...)
#endif
#ifdef HEARING_AID_KERNEL
	#ifndef GPIO_T3_ENABLE_PIN
	#define GPIO_T3_ENABLE_PIN 46 //temp setting
	#endif
#endif

//#ifndef GPIO_SPEAKER_EN_PIN
//#define GPIO_SPEAKER_EN_PIN (104)
//#endif



extern void Yusu_Sound_AMP_Switch(BOOL enable);

bool Speaker_Init(void)
{
   printk("+Speaker_Init Success");
   mt_set_gpio_mode(GPIO_SPEAKER_EN_PIN,GPIO_MODE_00);  // gpio mode
   mt_set_gpio_pull_enable(GPIO_SPEAKER_EN_PIN,GPIO_PULL_ENABLE);
   #ifdef HEARING_AID_KERNEL
   PRINTT3("@ZML+Init T3");
   mt_set_gpio_mode(GPIO_T3_ENABLE_PIN,GPIO_MODE_00);  // 46 is temp setting
   mt_set_gpio_pull_enable(GPIO_T3_ENABLE_PIN,GPIO_PULL_ENABLE);
   mt_set_gpio_out(GPIO_T3_ENABLE_PIN,GPIO_OUT_ZERO); // low
   PRINTT3("@ZML-Init T3");
   #endif
   printk("-Speaker_Init Success");
   return true;
}

void Sound_SpeakerL_SetVolLevel(int level)
{
   PRINTK(" Sound_SpeakerL_SetVolLevel level=%d\n",level);
}

void Sound_SpeakerR_SetVolLevel(int level)
{
   PRINTK(" Sound_SpeakerR_SetVolLevel level=%d\n",level);
}

void Sound_Speaker_Turnon(int channel)
{
    PRINTK("Sound_Speaker_Turnon channel = %d\n",channel);
    mt_set_gpio_dir(GPIO_SPEAKER_EN_PIN,GPIO_DIR_OUT); // output
    mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ONE); // high
    msleep(90); //delay 90ms to wait for speaker ready
    
}

void Sound_Speaker_Turnoff(int channel)
{
    PRINTK("Sound_Speaker_Turnoff channel = %d\n",channel);

    //mt_set_gpio_dir(GPIO_SPEAKER_EN_PIN,GPIO_DIR_IN); // input
    mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ZERO); // low   
}

void Sound_Speaker_SetVolLevel(int level)
{
    Speaker_Volume =level;

}


void Sound_Headset_Turnon(void)
{

}
void Sound_Headset_Turnoff(void)
{

}


static char *ExtFunArray[] =
{
    "InfoMATVAudioStart",
    "InfoMATVAudioStop",
    #ifdef HEARING_AID_KERNEL
    "infoHearingAidEnable",
    "infoHearingAidDisable",
    #endif
    "End",
};

#ifdef HEARING_AID_KERNEL
void Hearing_Aid_Switch(char enable)
{
     PRINTT3("@ZML +Hearing_Aid_Switch enable=%d",enable);
	if(1==enable)
	{
		PRINTT3("@ZML Enable GPIO");
		mt_set_gpio_dir(GPIO_T3_ENABLE_PIN,GPIO_DIR_OUT); // output
   		mt_set_gpio_out(GPIO_T3_ENABLE_PIN,GPIO_OUT_ONE); // high
	}else
	{
		PRINTT3("@ZML Disable GPIO");
   		mt_set_gpio_out(GPIO_T3_ENABLE_PIN,GPIO_OUT_ZERO); // low
	}
     PRINTT3("@ZML -Hearing_Aid_Switch enable=%d",enable);
}

#endif


kal_int32 Sound_ExtFunction(const char* name, void* param, int param_size)
{
	int i = 0;
	int funNum = -1;

	//Search the supported function defined in ExtFunArray
	while(strcmp("End",ExtFunArray[i]) != 0 ) {		//while function not equal to "End"
		
	    if (strcmp(name,ExtFunArray[i]) == 0 ) {		//When function name equal to table, break
	    	funNum = i;
	    	break;
	    }
	    i++;
	}

	switch (funNum) {
	    case 0:			//InfoMATVAudioStart
	        printk("RunExtFunction InfoMATVAudioStart \n");
	        break;

	    case 1:			//InfoMATVAudioStop
	        printk("RunExtFunction InfoMATVAudioStop \n");
	        break;
	 #ifdef HEARING_AID_KERNEL
	    case 2:
			PRINTT3("@ZML  Enable T3");
			Hearing_Aid_Switch(1);
			break;
	    case 3:
			PRINTT3("@ZML Disable T3");
			Hearing_Aid_Switch(0);
			break;
	 #endif

	    default:
	    	 break;
	}

	return 1;
}


