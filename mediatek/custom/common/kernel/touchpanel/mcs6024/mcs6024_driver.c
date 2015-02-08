
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif 
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <mach/mt6573_boot.h>


#include "tpd_custom_mcs6024.h"
#include "tpd.h"
#include <cust_eint.h>

#ifndef TPD_NO_GPIO 
#include "cust_gpio_usage.h"
#endif

//if the TP has external power with GPIO pin,need define TPD_HAVE_POWER_ON_OFF in tpd_custom_mcs6024.h
#define TPD_HAVE_POWER_ON_OFF

#define MAX_POINT 2



struct touch_info {
    int x[MAX_POINT], y[MAX_POINT];
    int p[MAX_POINT];
	int TouchpointFlag;
    int VirtualKeyFlag;
};

static int raw_x[MAX_POINT], raw_y[MAX_POINT];
struct touch_info cinfo, sinfo;



extern struct tpd_device *tpd;
extern int tpd_show_version;
//extern int tpd_debuglog;

static int boot_mode = 0;
static int tpd_flag = 0;
static int tpd_halt=0;
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

static void tpd_eint_interrupt_handler(void);
static int touch_event_handler(void *unused);
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int tpd_i2c_remove(struct i2c_client *client);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);

static struct i2c_client *i2c_client = NULL;
static const struct i2c_device_id tpd_i2c_id[] = {{"mtk-tpd",0},{}};
static unsigned short force[] = {0, 0x20, I2C_CLIENT_END,I2C_CLIENT_END};
static const unsigned short * const forces[] = { force, NULL };
static struct i2c_client_address_data addr_data = { .forces = forces,};
struct i2c_driver tpd_i2c_driver = {                       
    .probe = tpd_i2c_probe,                                   
    .remove = tpd_i2c_remove,                           
    .detect = tpd_i2c_detect,                           
    .driver.name = "mtk-tpd", 
    .id_table = tpd_i2c_id,                             
    .address_data = &addr_data,                        
}; 

static void tpd_hw_enable(void)
{
    /* CTP_LDO */
    mt_set_gpio_mode(GPIO38, 0);
	mt_set_gpio_dir(GPIO38, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO38, GPIO_OUT_ONE);

    /* CTP_EN */
	//mt_set_gpio_mode(GPIO21, 0);
	//mt_set_gpio_dir(GPIO21, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO21, GPIO_OUT_ONE);
    mdelay(20);
}

static void tpd_hw_disable(void)
{
    /* CTP_EN */
    //mt_set_gpio_mode(GPIO21, 0);
	//mt_set_gpio_dir(GPIO21, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO21, GPIO_OUT_ZERO);
    
    /* CTP_LDO */
    mt_set_gpio_mode(GPIO38, 0);
	mt_set_gpio_dir(GPIO38, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO38, GPIO_OUT_ZERO);
}

static void tpd_i2c_master_send(struct i2c_client *client,const char *buf ,int count)
{
	client->addr = client->addr & I2C_MASK_FLAG;
	i2c_master_send(client,(const char*)buf,count);
}

static  int tpd_i2c_master_rs_send(struct i2c_client *client,const char *buf ,int count)
{
	int ret;

	client->addr = (client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG; 
	ret = i2c_master_send(client,(const char*)buf,count);
	return ret;
}


static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
    strcpy(info->type, "mtk-tpd");
    return 0;
}

static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {             
	int err = 0,i;
	uint8_t Firmware_version[3] = {0x20,0x00,0x00};
	i2c_client = client;

	#ifdef TPD_HAVE_POWER_ON_OFF
	//for power on sequence
	tpd_hw_enable();

	//for power on sequence
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	
	#endif

	msleep(20);
	
	#if 0
	for(i = 0;i<5;i++)
	{
		if(tpd_i2c_master_rs_send(i2c_client,Firmware_version,3<<8|1) < 0)
		{
				TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
		}
		else
		{
			TPD_DMESG(" mcs6024 Hardware version is %x\n",Firmware_version[0]);
			TPD_DMESG(" mcs6024 Firmware version is %x\n",Firmware_version[1]);
			TPD_DMESG(" mcs6024 Panel Type  is %x\n",Firmware_version[2]);
			break;
		}
	}
	if(i == 5)
	{
		TPD_DMESG("mcs6024 tpd_i2c_probe fail\n");
		return -1;
	}
	#endif

	
		
	  thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
    if (IS_ERR(thread)) { 
        err = PTR_ERR(thread);
        TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", err);
    }    
    

	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	
   tpd_load_status = 1;
    return 0;
}

void tpd_eint_interrupt_handler(void) { 
	//if(tpd_em_debuglog==1) 
		//TPD_DMESG("[mtk-tpd], %s\n", __FUNCTION__);
    TPD_DEBUG_PRINT_INT; tpd_flag=1; wake_up_interruptible(&waiter);
} 
static int tpd_i2c_remove(struct i2c_client *client) {return 0;}

void tpd_down(int raw_x, int raw_y, int x, int y, int p) {	
    input_report_abs(tpd->dev, ABS_PRESSURE, p);
    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, p);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    TPD_DEBUG("D[%4d %4d %4d] ", x, y, p);
    input_mt_sync(tpd->dev);
    TPD_DOWN_DEBUG_TRACK(x,y);
    TPD_EM_PRINT(raw_x, raw_y, x, y, p, 1);
}

int tpd_up(int raw_x, int raw_y, int x, int y) {
        input_report_abs(tpd->dev, ABS_PRESSURE, 0);
        input_report_key(tpd->dev, BTN_TOUCH, 0);
        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
        input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
        input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
        TPD_DEBUG("U[%4d %4d %4d] ", x, y, 0);
        input_mt_sync(tpd->dev);
        TPD_UP_DEBUG_TRACK(x,y);
        TPD_EM_PRINT(raw_x, raw_y, x, y, 0, 0);
        return 1;
}


int tpd_gettouchinfo(struct touch_info *cinfo) {
	uint8_t status = 0x10;
	uint8_t  address_p1[4] = {0x11,0x00,0x00,0x00};
	uint8_t  address_p2[4] = {0x19,0x00,0x00,0x00};
	uint8_t i;

    sinfo.VirtualKeyFlag = cinfo->VirtualKeyFlag;
	cinfo->TouchpointFlag = 0;
    cinfo->VirtualKeyFlag= 0;
		
	tpd_i2c_master_rs_send(i2c_client,&status,(1<<8)|1);
	cinfo->TouchpointFlag = status&0x03;   //For multi-touch(three point)
	TPD_DEBUG("TouchpointFlag = %x\n",status);

    status = 0x18;
    tpd_i2c_master_rs_send(i2c_client,&status,(1<<8)|1);
    cinfo->VirtualKeyFlag = status&0x0F;
    TPD_DEBUG("VirtualKeyFlag = %x\n",status);
    
	if((cinfo->TouchpointFlag == 0) && ((cinfo->VirtualKeyFlag & 0x03) == 0))
		return 1;  //No touch

	if(cinfo->TouchpointFlag == 0x01) // 1 point
	{
		tpd_i2c_master_rs_send(i2c_client,address_p1,(4<<8)|1);
		cinfo->x[0] =address_p1[1]|((address_p1[0]&0x3)<<8);
		cinfo->y[0]= address_p1[3]|((address_p1[2]&0x3)<<8);
	}
	if(cinfo->TouchpointFlag == 0x02) 
	{
		tpd_i2c_master_rs_send(i2c_client,address_p2,(4<<8)|1);
		cinfo->x[1] =address_p2[1]|((address_p2[0]&0x3)<<8);
		cinfo->y[1]= address_p2[3]|((address_p2[2]&0x3)<<8);
	}
	if(cinfo->TouchpointFlag == 0x03) 
	{

		tpd_i2c_master_rs_send(i2c_client,address_p1,(4<<8)|1);
		tpd_i2c_master_rs_send(i2c_client,address_p2,(4<<8)|1);
		cinfo->x[0] =address_p1[1]|((address_p1[0]&0x3)<<8);
		cinfo->y[0]= address_p1[3]|((address_p1[2]&0x3)<<8);
		cinfo->x[1] =address_p2[1]|((address_p2[0]&0x3)<<8);
		cinfo->y[1]= address_p2[3]|((address_p2[2]&0x3)<<8);

	}

	//if(tpd_debuglog==1) {
		//TPD_DMESG("point data [%d %d %d  %d]\n",cinfo->x[0],cinfo->y[0],cinfo->x[1],cinfo->y[1]);
	//}
	
	TPD_DEBUG("point data [%d %d %d  %d]\n",address_p1[0],address_p1[1],address_p1[2],address_p1[3]);	
	TPD_DEBUG("point data [%d %d %d  %d]\n",cinfo->x[0],cinfo->y[0],cinfo->x[1],cinfo->y[1]);
		
	return 0;	
}


static int touch_event_handler(void *unused) {
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };

    int index;
	uint8_t Firmware_version[3] = {0x20,0x00,0x00};

    sched_setscheduler(current, SCHED_RR, &param);
    do {
        set_current_state(TASK_INTERRUPTIBLE);
        if (!kthread_should_stop()) {
            TPD_DEBUG_CHECK_NO_RESPONSE;
            do {
				while (tpd_halt) {tpd_flag = 0;sinfo.TouchpointFlag=0; msleep(20);}
               		wait_event_interruptible(waiter,tpd_flag!=0);
					tpd_flag = 0;
            } while(0);

            TPD_DEBUG_SET_TIME;
        }
       		 set_current_state(TASK_RUNNING);

			 if (tpd_show_version) {
	            tpd_show_version = 0;
			if(tpd_i2c_master_rs_send(i2c_client,Firmware_version,3<<8|1) < 0)
			{
					TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
			}
			else
			{
				TPD_DMESG(" mcs6024 Hardware version is %x\n",Firmware_version[0]);
				TPD_DMESG(" mcs6024 Firmware version is %x\n",Firmware_version[1]);
				TPD_DMESG(" mcs6024 Panel Type  is %x\n",Firmware_version[2]);
			}
		}
		
		 if((tpd_gettouchinfo(&cinfo)) 
            && (sinfo.TouchpointFlag==0) 
            && ((sinfo.VirtualKeyFlag & 0x03) == 0)) 
		 	continue; 
	
		TPD_DEBUG("sinfo.TouchpointFlag = %d\n",sinfo.TouchpointFlag);
		TPD_DEBUG("cinfo.TouchpointFlag = %d\n",cinfo.TouchpointFlag);
        TPD_DEBUG("sinfo.VirtualKeyFlag = %d\n",sinfo.VirtualKeyFlag);
		TPD_DEBUG("cinfo.VirtualKeyFlag = %d\n",cinfo.VirtualKeyFlag);

        #ifdef TPD_HAVE_BUTTON
        int index = (cinfo.VirtualKeyFlag >> 2) & 0x03;
        if (index >= TPD_KEY_COUNT)
            continue;
        
        if ((cinfo.VirtualKeyFlag & 0x03) == 1) {
            if ((sinfo.VirtualKeyFlag & 0x03) == 0) {
                tpd_down(tpd_keys_dim_local[index][0],tpd_keys_dim_local[index][1], tpd_keys_dim_local[index][0],tpd_keys_dim_local[index][1], 128);
                if (boot_mode != NORMAL_BOOT) 
                    tpd_button(tpd_keys_dim_local[index][0],tpd_keys_dim_local[index][1], 1);
            }
        } else {
            if ((sinfo.VirtualKeyFlag & 0x03) == 1) {
                tpd_up(tpd_keys_dim_local[index][0],tpd_keys_dim_local[index][1], tpd_keys_dim_local[index][0],tpd_keys_dim_local[index][1]);
                if (boot_mode != NORMAL_BOOT) 
                    tpd_button(tpd_keys_dim_local[index][0],tpd_keys_dim_local[index][1], 0);
            }
        }
        #endif
		 
		 for(index = 0;index<MAX_POINT;index++)
		 	{
				if(cinfo.TouchpointFlag&(1<<index))
				{
					raw_x[index] = cinfo.x[index];
					raw_y[index] = cinfo.y[index];
					tpd_down(raw_x[index], raw_y[index], cinfo.x[index],cinfo.y[index], 128);
					sinfo.x[index] = cinfo.x[index];
					sinfo.y[index] = cinfo.y[index];
					sinfo.TouchpointFlag |=(1<<index);
				}
				else
				{
					if(sinfo.TouchpointFlag&(1<<index))
					{
						tpd_up(raw_x[index], raw_y[index], sinfo.x[index], sinfo.y[index]);
						sinfo.TouchpointFlag &=~(1<<index);
					}

				}
		 	}

		 input_sync(tpd->dev);
		
    } while (!kthread_should_stop());
    return 0;
}


int tpd_local_init(void) 
{
	//if(tpd_debuglog==1) {
		//TPD_DMESG("[mtk-tpd] %s\n", __FUNCTION__); 
	//}
    boot_mode = get_boot_mode();
    // Software reset mode will be treated as normal boot
    if(boot_mode==3) boot_mode = NORMAL_BOOT;
    
     if(i2c_add_driver(&tpd_i2c_driver)!=0) {
      TPD_DMESG("unable to add i2c driver.\n");
      return -1;
    }
#ifdef TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
		TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
		tpd_type_cap = 1;
    return 0;
}

/* Function to manage low power suspend */
void tpd_suspend(struct early_suspend *h)
{
	//if(tpd_debuglog==1) {
		//TPD_DMESG("[mtk-tpd] %s\n", __FUNCTION__); 
	//}
	 tpd_halt = 1;
   	 mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	uint8_t bfffer[2] = {0x01,0x00};
	tpd_i2c_master_rs_send(i2c_client,bfffer,1<<8|1);
	bfffer[1] &= ~0x0E;
	tpd_i2c_master_send(i2c_client,bfffer,2);
	
#ifdef TPD_HAVE_POWER_ON_OFF
	/*mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);*/

    tpd_hw_disable();
#endif

}

/* Function to manage power-on resume */
void tpd_resume(struct early_suspend *h) 
{

	uint8_t bfffer[2] = {0x01,0x02};

	//if(tpd_debuglog==1) {
		//TPD_DMESG("[mtk-tpd] %s\n", __FUNCTION__); 
	//}
	
#ifdef TPD_HAVE_POWER_ON_OFF
	mt_set_gpio_mode(181, 1);
	mt_set_gpio_mode(182, 1);
	/*mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);*/

    tpd_hw_enable();
    
	tpd_i2c_master_send(i2c_client,bfffer,2);
	msleep(10);
#endif

	tpd_i2c_master_rs_send(i2c_client,bfffer,1<<8|1);
	bfffer[1] &= ~0x0E;
	bfffer[1] |= 0x02;
	tpd_i2c_master_send(i2c_client,bfffer,2);
	msleep(5);
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
    tpd_halt = 0;
}

static struct tpd_driver_t tpd_device_driver = {
		.tpd_device_name = "mcs6024",
		.tpd_local_init = tpd_local_init,
		.suspend = tpd_suspend,
		.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
		.tpd_have_button = 1,
#else
		.tpd_have_button = 0,
#endif		
};
/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
    printk("MediaTek mcs6024 touch panel driver init\n");
		if(tpd_driver_add(&tpd_device_driver) < 0)
			TPD_DMESG("add generic driver failed\n");
    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
    TPD_DMESG("MediaTek mcs6024 touch panel driver exit\n");
    //input_unregister_device(tpd->dev);
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

