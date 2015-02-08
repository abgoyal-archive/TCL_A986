

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#ifdef MT6516
#include <mach/mt6516_devs.h>
#include <mach/mt6516_typedefs.h>
#include <mach/mt6516_gpio.h>
#include <mach/mt6516_pll.h>
#endif

#ifdef MT6573
#include <mach/mt6573_devs.h>
#include <mach/mt6573_typedefs.h>
#include <mach/mt6573_gpio.h>
#include <mach/mt6573_pll.h>
#endif
#ifdef MT6516
#define POWER_NONE_MACRO MT6516_POWER_NONE
#endif

#ifdef MT6573
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "tmd2771.h"
#include <linux/wakelock.h>

struct wake_lock      ps_wake_lock;
/*----------------------------------------------------------------------------*/

#define TMD2771_DEV_NAME     "TMD2771"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)                 
extern void MT6516_EINTIRQUnmask(unsigned int line);
extern void MT6516_EINTIRQMask(unsigned int line);
extern void MT6516_EINT_Set_Polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void MT6516_EINT_Set_HW_Debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 MT6516_EINT_Set_Sensitivity(kal_uint8 eintno, kal_bool sens);
extern void MT6516_EINT_Registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
/*----------------------------------------------------------------------------*/
static struct i2c_client *tmd2771_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id tmd2771_i2c_id[] = {{TMD2771_DEV_NAME,0},{}};
/*the adapter id & i2c address will be available in customization*/
static unsigned short tmd2771_force[] = {0x02, 0X72, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const tmd2771_forces[] = { tmd2771_force, NULL };
static struct i2c_client_address_data tmd2771_addr_data = { .forces = tmd2771_forces,};
/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int tmd2771_i2c_remove(struct i2c_client *client);
static int tmd2771_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int tmd2771_i2c_resume(struct i2c_client *client);

static struct tmd2771_priv *g_tmd2771_ptr = NULL;
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct tmd2771_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct tmd2771_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct tmd2771_i2c_addr  addr;
    
    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver tmd2771_i2c_driver = {	
	.probe      = tmd2771_i2c_probe,
	.remove     = tmd2771_i2c_remove,
	.detect     = tmd2771_i2c_detect,
	.suspend    = tmd2771_i2c_suspend,
	.resume     = tmd2771_i2c_resume,
	.id_table   = tmd2771_i2c_id,
	.address_data = &tmd2771_addr_data,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = TMD2771_DEV_NAME,
	},
};

static struct tmd2771_priv *tmd2771_obj = NULL;
static struct platform_driver tmd2771_alsps_driver;
/*----------------------------------------------------------------------------*/
int tmd2771_get_addr(struct alsps_hw *hw, struct tmd2771_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void tmd2771_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "TMD2771")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "TMD2771")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}
/*----------------------------------------------------------------------------*/
static int tmd2771_enable_als(struct i2c_client *client, int enable)
{
		struct tmd2771_priv *obj = i2c_get_clientdata(client);
		u8 databuf[2];	  
		int res = 0;
		u8 buffer[1];
		u8 reg_value[1];
	
		if(client == NULL)
		{
			APS_DBG("CLIENT CANN'T EQUL NULL\n");
			return -1;
		}
	
		buffer[0]=TMD2771_CMM_ENABLE;
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, reg_value, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		
		if(enable)
		{
			databuf[0] = TMD2771_CMM_ENABLE;	
			databuf[1] = reg_value[0] |0x0B;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->ps_deb_on, 1);
			atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
			APS_DBG("tmd2771 power on\n");
		}
		else
		{
			databuf[0] = TMD2771_CMM_ENABLE;	
			databuf[1] = reg_value[0] &0xFD;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->ps_deb_on, 0);
			APS_DBG("tmd2771 power off\n");
		}
		return 0;
		
	EXIT_ERR:
		APS_ERR("tmd2771_enable_ps fail\n");
		return res;
}

/*----------------------------------------------------------------------------*/
static int tmd2771_enable_ps(struct i2c_client *client, int enable)
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]=TMD2771_CMM_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	if(enable)
	{
		databuf[0] = TMD2771_CMM_ENABLE;    
		databuf[1] = reg_value[0] |0x0d;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		APS_DBG("tmd2771 power on\n");
	}
	else
	{
		databuf[0] = TMD2771_CMM_ENABLE;    
		databuf[1] = reg_value[0] &0xfb;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("tmd2771 power off\n");
	}
	return 0;
	
EXIT_ERR:
	APS_ERR("tmd2771_enable_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int tmd2771_enable(struct i2c_client *client, int enable)
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	
	if(enable)
	{
		databuf[0] = TMD2771_CMM_ENABLE;    
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_DBG("tmd2771 power on\n");
	}
	else
	{
		databuf[0] = TMD2771_CMM_ENABLE;    
		databuf[1] = 0x00;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("tmd2771 power off\n");
	}
	return 0;
	
EXIT_ERR:
	APS_ERR("tmd2771_enable fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static int tmd2771_check_and_clear_intr(struct i2c_client *client) 
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);
	int err;
	u8 addr;

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;


	return 0;
}
/*----------------------------------------------------------------------------*/
void tmd2771_eint_func(void)
{
	struct tmd2771_priv *obj = g_tmd2771_ptr;
	if(!obj)
	{
		return;
	}
	
	schedule_work(&obj->eint_work);
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int tmd2771_init_client(struct i2c_client *client)
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = TMD2771_CMM_ENABLE;    
	databuf[1] = 0x01;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}
	
	databuf[0] = TMD2771_CMM_ATIME;    
	databuf[1] = 0xC9;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}

	databuf[0] = TMD2771_CMM_PTIME;    
	databuf[1] = 0xFF;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}

	databuf[0] = TMD2771_CMM_WTIME;    
	databuf[1] = 0xEE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}

	databuf[0] = TMD2771_CMM_CONFIG;    
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}

	databuf[0] = TMD2771_CMM_PPCOUNT;    
	databuf[1] = 0x08;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}

	databuf[0] = TMD2771_CMM_CONTROL;    
	databuf[1] = 0x22;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2771_ERR_I2C;
	}
	return TMD2771_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

int tmd2771_read_als(struct i2c_client *client, u16 *data)
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);	 
	u16 c0_value, c1_value;	 
	u32 c0_nf, c1_nf;
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	u16 atio;
	u16 als_value;
	int res = 0;
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
//get adc channel 0 value
	buffer[0]=TMD2771_CMM_C0DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	buffer[0]=TMD2771_CMM_C0DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	c0_value = als_value_low[0] | (als_value_high[0]<<8);
	c0_nf = obj->als_modulus*c0_value;
	//APS_DBG("c0_value=%d, c0_nf=%d, als_modulus=%d\n", c0_value, c0_nf, obj->als_modulus);

//get adc channel 1 value
	buffer[0]=TMD2771_CMM_C1DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
		
	buffer[0]=TMD2771_CMM_C1DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
		
	c1_value = als_value_low[0] | (als_value_high[0]<<8);
	c1_nf = obj->als_modulus*c1_value;	
	//APS_DBG("c1_value=%d, c1_nf=%d, als_modulus=%d\n", c1_value, c1_nf, obj->als_modulus);

//caculate the light value lux
	if (c0_value < c1_value)
	{
		APS_DBG("als_value is invalid!!\n");
		return -1;
	}
	else if (c0_value > 50000)
	{
		*data = 65535;
	}
	else
		atio = (c1_nf*100)/c0_nf;

	//APS_DBG("atio = %d\n", atio);
	if(atio<30)
	{
		*data = (13*c0_nf - 24*c1_nf)/10000;
	}
	else if(atio>30 && atio<38)
	{ 
		*data = (16*c0_nf - 35*c1_nf)/10000;
	}
	else if(atio>38 && atio<45)
	{ 
		*data = (9*c0_nf - 17*c1_nf)/10000;
	}
	else if(atio>45 && atio<54)
	{ 
		*data = (6*c0_nf - 10*c1_nf)/10000;
	}
	else
		*data = 0;

	//APS_DBG("als_value_lux = %d\n", *data);
	
	return 0;	 

	
	
EXIT_ERR:
	APS_ERR("tmd2771_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/

static int tmd2771_get_als_value(struct tmd2771_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		//APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
int tmd2771_read_ps(struct i2c_client *client, u16 *data)
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);    
	u16 ps_value;    
	u8 ps_value_low[1], ps_value_high[1];
	u8 buffer[1];
	int res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]=TMD2771_CMM_PDATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	buffer[0]=TMD2771_CMM_PDATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	*data = ps_value_low[0] | (ps_value_high[0]<<8);
	//APS_DBG("ps_data=%d, low:%d  high:%d", *data, ps_value_low[0], ps_value_high[0]);
	return 0;    

EXIT_ERR:
	APS_ERR("tmd2771_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int tmd2771_get_ps_value(struct tmd2771_priv *obj, u16 ps)
{
	int val, mask = atomic_read(&obj->ps_mask);
	int invalid = 0;

	if(ps > atomic_read(&obj->ps_thd_val))
	{
		val = 0;  /*close*/
	}
	else
	{
		val = 1;  /*far away*/
	}
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	else if (obj->als > 50000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
		//APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}
static int tmd2771_open(struct inode *inode, struct file *file)
{
	file->private_data = tmd2771_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tmd2771_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tmd2771_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct tmd2771_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = tmd2771_enable_ps(obj->client, 1))
				{
					APS_ERR("enable ps fail: %d\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if(err = tmd2771_enable_ps(obj->client, 0))
				{
					APS_ERR("disable ps fail: %d\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if(err = tmd2771_read_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}
			
			dat = tmd2771_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if(err = tmd2771_read_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = tmd2771_enable_als(obj->client, 1))
				{
					APS_ERR("enable als fail: %d\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if(err = tmd2771_enable_als(obj->client, 0))
				{
					APS_ERR("disable als fail: %d\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if(err = tmd2771_read_als(obj->client, &obj->als))
			{
				goto err_out;
			}

			dat = tmd2771_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if(err = tmd2771_read_als(obj->client, &obj->als))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations tmd2771_fops = {
	.owner = THIS_MODULE,
	.open = tmd2771_open,
	.release = tmd2771_release,
	.ioctl = tmd2771_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tmd2771_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &tmd2771_fops,
};
/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);    
	int err;
	APS_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		if(err = tmd2771_enable_als(client, 0))
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		atomic_set(&obj->ps_suspend, 1);
		if(err = tmd2771_enable_ps(client, 0))
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		tmd2771_power(obj->hw, 0);
                tmd2771_enable(client, 0);/* power off */
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_resume(struct i2c_client *client)
{
	struct tmd2771_priv *obj = i2c_get_clientdata(client);        
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
 
        tmd2771_enable(client, 1);/* power on */
	tmd2771_power(obj->hw, 1);
	if(err = tmd2771_init_client(client))
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = tmd2771_enable_als(client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		if(err = tmd2771_enable_ps(client, 1))
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}
	APS_LOG("tmd2771 add wake lock\n");
	wake_lock_timeout(&(ps_wake_lock),100);

	return 0;
}
/*----------------------------------------------------------------------------*/
static void tmd2771_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct tmd2771_priv *obj = container_of(h, struct tmd2771_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	atomic_set(&obj->als_suspend, 1);    
	if(err = tmd2771_enable_als(obj->client, 0))
	{
		APS_ERR("disable als fail: %d\n", err); 
	}
}
/*----------------------------------------------------------------------------*/
static void tmd2771_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct tmd2771_priv *obj = container_of(h, struct tmd2771_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = tmd2771_enable_als(obj->client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
}

int tmd2771_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct tmd2771_priv *obj = (struct tmd2771_priv *)self;
	
	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
					if(err = tmd2771_enable_ps(obj->client, 1))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					if(err = tmd2771_enable_ps(obj->client, 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;	
				tmd2771_read_ps(obj->client, &obj->ps);
				
				sensor_data->values[0] = tmd2771_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int tmd2771_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct tmd2771_priv *obj = (struct tmd2771_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
					if(err = tmd2771_enable_als(obj->client, 1))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if(err = tmd2771_enable_als(obj->client, 0))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				tmd2771_read_als(obj->client, &obj->als);
								
				sensor_data->values[0] = tmd2771_get_als_value(obj, obj->als);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, TMD2771_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tmd2771_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	tmd2771_obj = obj;

	obj->hw = get_cust_alsps_hw();
	tmd2771_get_addr(obj->hw, &obj->addr);

	//INIT_WORK(&obj->eint_work, tmd2771_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);   
	obj->als_modulus = (400*100)/(16*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	
	tmd2771_i2c_client = client;

	
	if(err = tmd2771_init_client(client))
	{
		goto exit_init_failed;
	}
	APS_LOG("tmd2771_init_client() OK!\n");

	if(err = misc_register(&tmd2771_device))
	{
		APS_ERR("tmd2771_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	obj_ps.self = tmd2771_obj;
	obj_ps.polling = 1;
	obj_ps.sensor_operate = tmd2771_ps_operate;
	if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = tmd2771_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = tmd2771_als_operate;
	if(err = hwmsen_attach(ID_LIGHT, &obj_als))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = tmd2771_early_suspend,
	obj->early_drv.resume   = tmd2771_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

	wake_lock_init(&(ps_wake_lock),WAKE_LOCK_SUSPEND,"ps_wake_lock");
	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&tmd2771_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
	exit_kfree:
	kfree(obj);
	exit:
	tmd2771_i2c_client = NULL;           
//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int tmd2771_i2c_remove(struct i2c_client *client)
{
	int err;	
	if(err = misc_deregister(&tmd2771_device))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	tmd2771_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int tmd2771_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	tmd2771_power(hw, 1);    
	tmd2771_force[0] = hw->i2c_num;
	tmd2771_force[1] = hw->i2c_addr[0];
	APS_DBG("I2C = %d, addr =0x%x\n",tmd2771_force[0],tmd2771_force[1]);
	if(i2c_add_driver(&tmd2771_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tmd2771_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	tmd2771_power(hw, 0);    
	i2c_del_driver(&tmd2771_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver tmd2771_alsps_driver = {
	.probe      = tmd2771_probe,
	.remove     = tmd2771_remove,    
	.driver     = {
		.name  = "als_ps",
		.owner = THIS_MODULE,
	}
};
/*----------------------------------------------------------------------------*/
static int __init tmd2771_init(void)
{
	APS_FUN();
	if(platform_driver_register(&tmd2771_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit tmd2771_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&tmd2771_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(tmd2771_init);
module_exit(tmd2771_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("tmd2771 driver");
MODULE_LICENSE("GPL");
