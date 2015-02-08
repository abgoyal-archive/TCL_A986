
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"

#if defined(CONFIG_ARCH_MT6516)
    #include <mach/mt6516_typedefs.h>
    #include <mach/mt6516_gpt_sw.h>
#elif defined(CONFIG_ARCH_MT6573)
    #include <mach/mt6573_typedefs.h>
    #include <mach/mt6573_gpt.h>
#else
    #error "unknown arch"
#endif

/* device name and major number */
#define STROBE_DEVNAME            "leds_strobe"

#define DELAY_MS(ms) {mdelay(ms);}//unit: ms(10^-3)
#define DELAY_US(us) {mdelay(us);}//unit: us(10^-6)
#define DELAY_NS(ns) {mdelay(ns);}//unit: ns(10^-9)
#define PFX "[LEDS_STROBE]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    printk(KERN_INFO PFX "%s: " fmt, __FUNCTION__ ,##arg)

#define PK_WARN(fmt, arg...)        printk(KERN_WARNING PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      printk(KERN_NOTICE PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        printk(KERN_INFO PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              printk(PFX "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) printk(PFX fmt, ##arg)

//#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_TRC PK_DBG_NONE //PK_TRC_FUNC
#define PK_VER PK_DBG_NONE //PK_TRC_VERBOSE
#define IH_DBG PK_DBG_NONE
#define PK_ERR(fmt, arg...)         printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)
#else
#define PK_DBG(a,...)
#define PK_TRC(a,...)
#define PK_VER(a,...)
#define IH_DBG(a,...)
#define PK_ERR(a,...)
#endif

struct strobe_data{
    spinlock_t lock;
    wait_queue_head_t read_wait;
    struct semaphore sem;
};

static struct strobe_data strobe_private;

static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = FALSE;
static u32 strobe_width = 0; //0 is disable
static eFlashlightState strobe_eState = FLASHLIGHTDRV_STATE_PREVIEW;
/*extern functions*/

#define GPIO_CAMERA_FLASH_EN GPIO48
static void flashlight_timer_func(unsigned long param)
{
}

    ssize_t gpio_FL_Init(void) {
		mt_set_gpio_mode(GPIO48, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO48, GPIO_DIR_OUT);
        return 0;
    }
    ssize_t gpio_FL_Uninit(void) {
        return 0;
    }
    ssize_t gpio_FL_Enable(void) {
        //Enable
        if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN,GPIO_OUT_ONE))
			PK_DBG("[constant_flashlight] set gpio failed!! \n");
        return 0;
    }
    ssize_t gpio_FL_Disable(void) {
	if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN,GPIO_OUT_ZERO))
		PK_DBG("[constant_flashlight] set gpio failed!! \n");
        return 0;
    }
    ssize_t gpio_FL_dim_duty(kal_uint8 duty) {
        //N/A
        
        return 0;
    }

//abstract interface
#define FL_Init gpio_FL_Init
#define FL_Uninit gpio_FL_Uninit
#define FL_Enable gpio_FL_Enable
#define FL_Disable gpio_FL_Disable
#define FL_dim_duty gpio_FL_dim_duty

static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
    int i4RetValue = 0;
    int iFlashType = (int)FLASHLIGHT_NONE;

    //when ON state , only disable command is permitted.
    if ( (TRUE == g_strobe_On) && !((FLASHLIGHTIOC_T_ENABLE == cmd) && (0 == arg)) )

    {
        PK_DBG("[constant_flashlight]  is already ON OR check parameters!\n");
        return i4RetValue;
    }

    switch(cmd)
    {
    	case FLASHLIGHTIOC_T_ENABLE :
            if (arg && strobe_width) {
                if(FL_Enable()){
                    PK_ERR("FL_Enable fail!\n");
                    goto strobe_ioctl_error;
                }
                g_strobe_On = TRUE;
            }
            else{
                if(FL_Disable()){
                    PK_ERR("FL_Disable fail!\n");
                    goto strobe_ioctl_error;
                }
				g_strobe_On = FALSE;
            }
    		break;
        case FLASHLIGHTIOC_T_LEVEL:
            if(arg>32)
				{arg=32;}
            strobe_width = arg;
            PK_DBG("level:%d \n",(int)arg);
            if (arg > 0)
            {
                if(FL_dim_duty((kal_int8)arg - 1)) {
                    //0(weak)~31(strong)
                    PK_ERR("FL_dim_duty fail!\n");
                    //i4RetValue = -EINVAL;
                    goto strobe_ioctl_error;
                }
            }
            break;
        case FLASHLIGHTIOC_T_FLASHTIME:
            strobe_Timeus = (u32)arg;
            PK_DBG("strobe_Timeus:%d \n",(int)strobe_Timeus);
            break;
        case FLASHLIGHTIOC_T_STATE:
            strobe_eState = (eFlashlightState)arg;
            break;
        case FLASHLIGHTIOC_G_FLASHTYPE:
            iFlashType = FLASHLIGHT_LED_CONSTANT;
            if(copy_to_user((void __user *) arg , (void*)&iFlashType , _IOC_SIZE(cmd)))
            {
                PK_DBG("[strobe_ioctl] ioctl copy to user failed\n");
                return -EFAULT;
            }
            break;
            
    	default :
    		PK_DBG("No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }

    return i4RetValue;

strobe_ioctl_error:
    PK_DBG("Error or ON state!\n");
    return -EPERM;
}

static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    if (0 == strobe_Res) {
        FL_Init();
        
		//enable HW here if necessary
        if(FL_dim_duty(0)) {
            //0(weak)~31(strong)
            PK_ERR("FL_dim_duty fail!\n");
            i4RetValue = -EINVAL;
        }
    }//

    spin_lock_irq(&strobe_private.lock);

    if(strobe_Res)
    {
        PK_ERR("busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }

    //LED On Status
    g_strobe_On = FALSE;

    spin_unlock_irq(&strobe_private.lock);


    return i4RetValue;
}

static int constant_flashlight_release(void *pArg)
{
    if (strobe_Res)
    {
        spin_lock_irq(&strobe_private.lock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        //LED On Status
        g_strobe_On = FALSE;

        spin_unlock_irq(&strobe_private.lock);

    	//un-init. HW here if necessary
        if(FL_dim_duty(0)) {
            PK_ERR("FL_dim_duty fail!\n");
        }

    	FL_Uninit();
    }

    return 0;
}

FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl,
};

MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc) { 
    if (pfFunc!=NULL) {
        *pfFunc=&constantFlashlightFunc;
    }
    return 0;
}
