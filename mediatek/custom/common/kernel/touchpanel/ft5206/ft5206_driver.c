
 
#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include "tpd_custom_ft5206.h"


#include "cust_gpio_usage.h"

 
 
extern struct tpd_device *tpd;
 
struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
 
 
static void tpd_eint_interrupt_handler(void);
 
 
 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);

 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
 

static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;

//#define TPD_CLOSE_POWER_IN_SLEEP

#define TPD_OK 0
//register define

#define DEVICE_MODE 0x00
#define GEST_ID 0x01
#define TD_STATUS 0x02

#define TOUCH1_XH 0x03
#define TOUCH1_XL 0x04
#define TOUCH1_YH 0x05
#define TOUCH1_YL 0x06

#define TOUCH2_XH 0x09
#define TOUCH2_XL 0x0A
#define TOUCH2_YH 0x0B
#define TOUCH2_YL 0x0C

#define TOUCH3_XH 0x0F
#define TOUCH3_XL 0x10
#define TOUCH3_YH 0x11
#define TOUCH3_YL 0x12

//register define

struct touch_info {
    int y[3];
    int x[3];
    int p[3];
    int count;
};
 
 static const struct i2c_device_id tpd_id[] = {{TPD_DEVICE,0},{}};
 unsigned short force[] = {0,0x70,I2C_CLIENT_END,I2C_CLIENT_END}; 
 static const unsigned short * const forces[] = { force, NULL };
 static struct i2c_client_address_data addr_data = { .forces = forces, };
 
 
 static struct i2c_driver tpd_i2c_driver = {
  .driver = {
	 .name = TPD_DEVICE,
	 .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = __devexit_p(tpd_remove),
  .id_table = tpd_id,
  .detect = tpd_detect,
  .address_data = &addr_data,
 };
 

static  void tpd_down(int x, int y, int p) {
	// input_report_abs(tpd->dev, ABS_PRESSURE, p);
	 input_report_key(tpd->dev, BTN_TOUCH, 1);
	 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	 //printk("D[%4d %4d %4d] ", x, y, p);
	 input_mt_sync(tpd->dev);
	 TPD_DOWN_DEBUG_TRACK(x,y);
 }
 
static  int tpd_up(int x, int y,int *count) {
	 if(*count>0) {
		 input_report_abs(tpd->dev, ABS_PRESSURE, 0);
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
		 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
		 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		 //printk("U[%4d %4d %4d] ", x, y, 0);
		 input_mt_sync(tpd->dev);
		 TPD_UP_DEBUG_TRACK(x,y);
		 (*count)--;
		 return 1;
	 } return 0;
 }

 static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
 {

	int i = 0;
	
	char data[30] = {0};

    u16 high_byte,low_byte;

	p_point_num = point_num;

	i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &(data[0]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[24]));
	TPD_DEBUG("FW version=%x]\n",data[24]);
	
	TPD_DEBUG("received raw data from touch panel as following:\n");
	TPD_DEBUG("[data[0]=%x,data[1]= %x ,data[2]=%x ,data[3]=%x ,data[4]=%x ,data[5]=%x]\n",data[0],data[1],data[2],data[3],data[4],data[5]);
	TPD_DEBUG("[data[9]=%x,data[10]= %x ,data[11]=%x ,data[12]=%x]\n",data[9],data[10],data[11],data[12]);
	TPD_DEBUG("[data[15]=%x,data[16]= %x ,data[17]=%x ,data[18]=%x]\n",data[15],data[16],data[17],data[18]);

	
	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if(data[0] & 0x70 != 0) return false; 

	/*get the number of the touch points*/
	point_num= data[2] & 0x0f;
	
	TPD_DEBUG("point_num =%d\n",point_num);
	
//	if(point_num == 0) return false;

	   TPD_DEBUG("Procss raw data...\n");

		
		for(i = 0; i < point_num; i++)
		{
			cinfo->p[i] = data[3+6*i] >> 6; //event flag 

	       /*get the X coordinate, 2 bytes*/
			high_byte = data[3+6*i];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i + 1];
			cinfo->x[i] = high_byte |low_byte;

				//cinfo->x[i] =  cinfo->x[i] * 480 >> 11; //calibra
		
			/*get the Y coordinate, 2 bytes*/
			
			high_byte = data[3+6*i+2];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i+3];
			cinfo->y[i] = high_byte |low_byte;

			  //cinfo->y[i]=  cinfo->y[i] * 800 >> 11;
		
			cinfo->count++;
			
		}
		TPD_DEBUG(" cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);	
		TPD_DEBUG(" cinfo->x[1] = %d, cinfo->y[1] = %d, cinfo->p[1] = %d\n", cinfo->x[1], cinfo->y[1], cinfo->p[1]);		
		TPD_DEBUG(" cinfo->x[2]= %d, cinfo->y[2]= %d, cinfo->p[2] = %d\n", cinfo->x[2], cinfo->y[2], cinfo->p[2]);	
		  
	 return true;

 };

 static int touch_event_handler(void *unused)
 {
  
    struct touch_info cinfo, pinfo;

	 struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	 sched_setscheduler(current, SCHED_RR, &param);
 
	 do
	 {
	  mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		 set_current_state(TASK_INTERRUPTIBLE); 
		  wait_event_interruptible(waiter,tpd_flag!=0);
						 
			 tpd_flag = 0;
			 
		 set_current_state(TASK_RUNNING);
		 

		  if (tpd_touchinfo(&cinfo, &pinfo)) {
		  TPD_DEBUG("point_num = %d\n",point_num);
		  
            if(point_num >0) {
                tpd_down(cinfo.x[0], cinfo.y[0], 1);
             if(point_num>1)
             	{
			 	tpd_down(cinfo.x[1], cinfo.y[1], 1);
			   if(point_num >2) tpd_down(cinfo.x[2], cinfo.y[2], 1);
             	}
                input_sync(tpd->dev);
				TPD_DEBUG("press --->\n");
				
            } else  {
            TPD_DEBUG("release --->\n"); 
                input_mt_sync(tpd->dev);
                input_sync(tpd->dev);
            }
        }

 }while(!kthread_should_stop());
 
	 return 0;
 }
 
 static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info) 
 {
	 strcpy(info->type, TPD_DEVICE);	
	  return 0;
 }
 
 static void tpd_eint_interrupt_handler(void)
 {
	 TPD_DEBUG("TPD interrupt has been triggered\n");
	 tpd_flag = 1;
	 wake_up_interruptible(&waiter);
	 
 }
 static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 
	int retval = TPD_OK;
	char data;
	i2c_client = client;

	#ifdef TPD_CLOSE_POWER_IN_SLEEP	 
	hwPowerDown(TPD_POWER_SOURCE,"TP");
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
	msleep(100);
	#else
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	msleep(100);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	#endif

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
 
	  mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	  mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	  mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
	  mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
 
	msleep(100);
 
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	   {
		   TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
		   return -1; 
	   }

	tpd_load_status = 1;

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	 if (IS_ERR(thread))
		 { 
		  retval = PTR_ERR(thread);
		  TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
		}

	TPD_DMESG("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
   return 0;
   
 }

 static int __devexit tpd_remove(struct i2c_client *client)
 
 {
   
	 TPD_DEBUG("TPD removed\n");
 
   return 0;
 }
 
 
 static int tpd_local_init(void)
 {

 
  TPD_DMESG("Focaltech FT5206 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
 
 
   if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
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
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
		TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
		tpd_type_cap = 1;
    return 0; 
 }

 static int tpd_resume(struct i2c_client *client)
 {
  int retval = TPD_OK;
 
   TPD_DEBUG("TPD wake up\n");
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP"); 
#else

	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	msleep(100);

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(1);  
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
   mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
	
	 return retval;
 }
 
 static int tpd_suspend(struct i2c_client *client, pm_message_t message)
 {
	 int retval = TPD_OK;
	 static char data = 0x3;
 
	 TPD_DEBUG("TPD enter sleep\n");
	 mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerDown(TPD_POWER_SOURCE,"TP");
#else
i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);

#endif
	 return retval;
 } 


 static struct tpd_driver_t tpd_device_driver = {
		 .tpd_device_name = "FT5206",
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
	 printk("MediaTek FT5206 touch panel driver init\n");
		 if(tpd_driver_add(&tpd_device_driver) < 0)
			 TPD_DMESG("add FT5206 driver failed\n");
	 return 0;
 }
 
 /* should never be called */
 static void __exit tpd_driver_exit(void) {
	 TPD_DMESG("MediaTek FT5206 touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
 }
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);


