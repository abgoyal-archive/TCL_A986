
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

#include <linux/dma-mapping.h>

#include <asm/types.h>
#include "tpd_custom_ft5306.h"

#include "tpd.h"
#include <cust_eint.h>

#ifndef TPD_NO_GPIO 
#include "cust_gpio_usage.h"
#endif


//add wangdongliang DMA
static unsigned char *tpDMABuf_va = NULL;
static u32 tpDMABuf_pa = NULL;
//end wangdongliang DMA


//if the TP has external power with GPIO pin,need define TPD_HAVE_POWER_ON_OFF in tpd_custom_mcs6024.h
#define TPD_HAVE_POWER_ON_OFF

#define MAX_POINT 2

static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;


//added wangdongliang

#define TP_UPGRADE 1
#define I2C_CTPM_ADDRESS 0x70
#define    FTS_PACKET_LENGTH        128

#if TP_UPGRADE
static unsigned char CTPM_TRULY_FW[]=
{
	#include "mojitolite_v0f_truly.h"
};

static unsigned char CTPM_MUDONG_FW[]=
{
	#include "mojitolite_v0c_mudong.h"
};

#endif

extern struct tpd_device *tpd;
extern int tpd_show_version;

char version[2][10] = {"Truly", "Mudong"};

static unsigned char uc_tp_factory_ID;

static int boot_mode = 0;
static int tpd_flag = 0;
static int tpd_halt=0;
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);


#define DEBUG 0
/*registers*/
#define RAW_DATA_BEGIN_ADDR			0x00
/* pressed event */
#define TOUCH_DOWN			0x00
#define TOUCH_UP			0x01
#define	HOLD				0x02
#define HIBERNATE			0x03

#define EVENT_DOWN			1
#define EVENT_UP			        0

#define MIN_X				0
#define MAX_X				319
#define MIN_Y				0
#define MAX_Y				479
#define KEY_Y_TOP		        485
#define KEY_Y_BOTTOM		530
#define KEY_MENU_XL		0
#define KEY_MENU_XR		50
#define KEY_HOME_XL		130
#define KEY_HOME_XR		140
#define KEY_SEARCH_XL		185
#define KEY_SEARCH_XR		200
#define KEY_BACK_XL		278
#define KEY_BACK_XR		290

static int virtual_key[4][2] = {{0,50},{130,140},{185,200},{278,290}};

#define MAX_TOUCH_POINTS	2	/*support two points pressed*/
#define MAX_RAW_DATA_BYTE   13
#define POINT_DATA_LENGTH		6

#define REPORT_TP(a, b) { \
	input_report_abs(a->input, ABS_MT_TRACKING_ID, b.touch_id); \
	input_report_abs(a->input, ABS_MT_TOUCH_MAJOR, b.pressed); \
	input_report_abs(a->input, ABS_MT_POSITION_X, b.x); \
	input_report_abs(a->input, ABS_MT_POSITION_Y, b.y); \
	input_mt_sync(a->input); \
}
struct point_data {
	u16 x;
	u16 y;
	unsigned int touch_event;
	unsigned int touch_id;
	unsigned int pressed;
};

struct tpd_data {
	struct input_dev *input;
	struct i2c_client *client;
	struct work_struct work;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	u32 irq;
	u8 *raw_data;
	struct point_data points[MAX_TOUCH_POINTS];
	unsigned int total_tp;
	unsigned int last_total_tp;
};

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
static unsigned short force[] = {0, 0x70, I2C_CLIENT_END,I2C_CLIENT_END};
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
//rst pin
	mt_set_gpio_mode(GPIO43, 0);
	mt_set_gpio_dir(GPIO43, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO43, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO43, GPIO_OUT_ZERO);
	mdelay(5);
	mt_set_gpio_out(GPIO43, GPIO_OUT_ONE);
	
//enable
	mt_set_gpio_mode(GPIO104, 0);
	mt_set_gpio_dir(GPIO104, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO104, GPIO_OUT_ONE);
    mdelay(20);
}

static void tpd_hw_disable(void)
{
    /* CTP_EN */

//rst pin
    mt_set_gpio_mode(GPIO43, 0);
	mt_set_gpio_dir(GPIO43, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO43, GPIO_OUT_ZERO);

//enable
	mt_set_gpio_mode(GPIO104, 0);
	mt_set_gpio_dir(GPIO104, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO104, GPIO_OUT_ZERO);
    
}

static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
    strcpy(info->type, "mtk-tpd");
    return 0;
}
//add wangdongliang


static int write_reg(u8 addr, u8 para)
{
	char buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = i2c_master_send(i2c_client, buf, 2);
	if (ret < 0){
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}
	return 0;
}

static int read_reg(u8 addr, unsigned char *pdata)
{
	int ret;
	unsigned char buf[2];
	struct  i2c_msg msgs[2];

	buf[0] = addr;               //register address

	i2c_master_send(i2c_client, &buf[0], 1);
	ret = i2c_master_recv(i2c_client, &buf[0], 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;
}


//end wangdongliang

#if TP_UPGRADE
ssize_t mt6573_dma_write_m_byte(unsigned char*returnData_va, u32 returnData_pa, int  len)
{

    int     ret=0;

	 i2c_client->addr = (i2c_client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
    if (len > 0){
        ret = i2c_master_send(i2c_client, returnData_pa, len);
        if (ret < 0) {
            printk(KERN_ERR"xxxxfocal write data error!! xxxx\n");
            return 0;
        }
	
    }
//	i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
	//printk("xxxxwrite transfer ok!!!!!!xxxx\n");
    return 1;
}


u8 cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
	u8 write_cmd[4] = {0};
	i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
//	i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
	return i2c_master_send(i2c_client, write_cmd, num);
}


int  fts_ctpm_fw_upgrade(unsigned char* pbt_buf, int dw_lenth)
{
	unsigned char reg_val[3] = {0};
	int i = 0;

	int  packet_number;
	int  j;
	int  temp;
	int  lenght;
	unsigned char  packet_buf[FTS_PACKET_LENGTH + 6];
	unsigned char  auc_i2c_write_buf[10];
	unsigned char bt_ecc;
	int      i_ret;

	printk("=====================fts_ctpm_fw_upgrade======================\n");
	/*********Step 1:Reset  CTPM *****/
	/*write 0xaa to register 0xfc*/
	write_reg(0xfc,0xaa);
	mdelay(50);
	/*write 0x55 to register 0xfc*/
	write_reg(0xfc,0x55);
	printk("[FT520X] Step 1: Reset CTPM test\n");

	mdelay(30);   

	/*********Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do
	{
		i ++;
	//	i_ret = ft5306_i2c_txdata(auc_i2c_write_buf, 2);
		i_ret = i2c_master_send(i2c_client, auc_i2c_write_buf, 2);
		mdelay(5);
	}while(i_ret <= 0 && i < 5 );
	printk("[FT520X] Step 2: Enter upgrade mode\n");
	
	/*********Step 3:check READ-ID***********************/        
	cmd_write(0x90,0x00,0x00,0x00,4);
	i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
	i2c_master_recv(i2c_client, &reg_val, 2);
	printk("[FT520X] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	//byte_read(reg_val,2);
	if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
	{
		printk("[FT520X] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	}
	else
	{
		return 2;
		//i_is_new_protocol = 1;
	}

	/*********Step 4:erase app*******************************/
	cmd_write(0x61,0x00,0x00,0x00,1);

	mdelay(1500);
	//cmd_write(0x63, 0x00, 0x00, 0x00, 1);
	//mdelay(100);
	printk("[FT520X] Step 4: erase. \n");

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	printk("[FT520X] Step 5: start upgrade. \n");
	dw_lenth = dw_lenth - 8;
	printk("####Packet length = 0x %x\n", dw_lenth);
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	tpDMABuf_va[0] = 0xbf;
	tpDMABuf_va[1] = 0x00;
	for (j=0;j<packet_number;j++)
	{
		temp = j * FTS_PACKET_LENGTH;
		tpDMABuf_va[2] = (u8)(temp>>8);
		tpDMABuf_va[3] = (u8)temp;
		lenght = FTS_PACKET_LENGTH;
		tpDMABuf_va[4] = (u8)(lenght>>8);
		tpDMABuf_va[5] = (u8)lenght;

		for (i=0;i<FTS_PACKET_LENGTH;i++)
		{
			tpDMABuf_va[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= tpDMABuf_va[6+i];
		}

		mt6573_dma_write_m_byte(tpDMABuf_va, tpDMABuf_pa, FTS_PACKET_LENGTH + 6);
		mdelay(50);
		if ((j * FTS_PACKET_LENGTH % 1024) == 0)
		{
			printk("[FT520X] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		tpDMABuf_va[2] = (u8)(temp>>8);
		tpDMABuf_va[3] = (u8)temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		tpDMABuf_va[4] = (u8)(temp>>8);
		tpDMABuf_va[5] = (u8)temp;

		for (i=0;i<temp;i++)
		{
			tpDMABuf_va[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= tpDMABuf_va[6+i];
		}

		mt6573_dma_write_m_byte(tpDMABuf_va, tpDMABuf_pa,temp+6);    
			i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;

		mdelay(30);
	}

	//send the last six byte
	for (i = 0; i<6; i++)
	{
		packet_buf[0] = 0xbf;
		packet_buf[1] = 0x00;
		temp = 0x6ffa + i;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;
		temp =1;
		packet_buf[4] = (u8)(temp>>8);
		packet_buf[5] = (u8)temp;
		packet_buf[6] = pbt_buf[ dw_lenth + i]; 
		bt_ecc ^= packet_buf[6];

		i2c_master_send(i2c_client, packet_buf, 7);    

		mdelay(40);
	}

	/*********Step 6: read out checksum***********************/
	/*send the operation head*/
//	reg_val[0] = 0x01;
	cmd_write(0xcc,0x00,0x00,0x00,1);
	i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
	i2c_master_recv(i2c_client, &reg_val, 1);
	printk("[FT520X] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
	if(reg_val[0] != bt_ecc)
	{
		printk("5 check sum error!!\n");
		return 5;
	}

	/*********Step 7: reset the new FW***********************/
	cmd_write(0x07,0x00,0x00,0x00,1);
	printk("[FT520X] Step 7: reset the new FW. \n");
	
	/*********Step 8: calibration TP ***********************/
	mdelay(300);          //—” ±100ms
	#if 0
	read_reg(0xfc, &reg_val);
	if (reg_val[0] == 1)
	{
           write_reg(0xfc, 4);
           mdelay(2500);      //—” ±2500ms
           
           do
           {
		read_reg(0xfc, &reg_val);
		mdelay(100);          //—” ±100ms
           }while (reg_val[0] != 1);            
	}
	write_reg(0x00, 0x00);
	printk("[FT520X] Step 8: calibration TP. \n");
	#endif

	return 0;
}

int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp[1];
    unsigned char i ;

    printk("[FTS] start auto CLB.\n");
    msleep(200);
    write_reg(0, 0x40);  
    mdelay(100);   //make sure already enter factory mode
    write_reg(2, 0x4);  //write command to start calibration
    mdelay(300);
    for(i=0;i<100;i++)
    {
        read_reg(0,uc_temp);
        if ( ((uc_temp[0]&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
        {
            break;
        }
        mdelay(200);
        printk("[FTS] waiting calibration %d\n",i);
        
    }
    printk("[FTS] calibration OK.\n");
    
    msleep(300);
    write_reg(0, 0x40);  //goto factory mode
    mdelay(100);   //make sure already enter factory mode
    write_reg(2, 0x5);  //store CLB result
    mdelay(300);
    write_reg(0, 0x0); //return to normal mode 
    msleep(300);
    printk("[FTS] store CLB result OK.\n");
    return 0;
}

int fts_ctpm_fw_upgrade_with_i_file(unsigned char *buf, long unsigned int  length)
{
	unsigned char*     pbt_buf = NULL;
	int i_ret;

	printk("=========FW upgrade========================\n");

	//=========FW upgrade========================*/
	pbt_buf = buf;
	/*call the upgrade function*/
	i_ret =  fts_ctpm_fw_upgrade(pbt_buf, length);
	if (i_ret != 0)
	{
      		 printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
       		//error handling ...
      	 	//TBD
 	  }
 	  else
   	{
      		printk("[FTS] upgrade successfully.\n");
       		fts_ctpm_auto_clb();  //start auto CLB
  	 }

   return i_ret;
}

unsigned char fts_ctpm_get_upg_ver(unsigned char* buff, unsigned long int length)
{
//	unsigned int ui_sz;
//	ui_sz = sizeof(CTPM_FW);
	if (length > 2)
	{
		return buff[length - 2];
	}
	else
	{
		//TBD, error handling?
		return 0xff; //default value
	}
}

#endif

#define    FTS_SETTING_BUF_LEN        128

//return factory ID
int fts_ctpm_get_panel_factory_setting(void)
{
    unsigned char uc_i2c_addr;             //I2C slave address (8 bit address)
    unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;	1----1.8v
    unsigned char uc_panel_factory_id;     //TP panel factory ID

    unsigned char buf[FTS_SETTING_BUF_LEN];
    unsigned char reg_val[2] = {0};
    unsigned char  auc_i2c_write_buf[10];
    unsigned char  packet_buf[FTS_SETTING_BUF_LEN + 6];
    int i = 0;
    int      i_ret;

    uc_i2c_addr = 0x70;
    uc_io_voltage = 0x0;
    uc_panel_factory_id = 0x5a;

    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    write_reg(0xfc,0xaa);
    mdelay(50);
     /*write 0x55 to register 0xfc*/
    write_reg(0xfc,0x55);
    printk("[FTS] Step 1: Reset CTPM test\n");
   
    mdelay(30);   

    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
		i ++;
	//	i_ret = ft5306_i2c_txdata(auc_i2c_write_buf, 2);
		i_ret = i2c_master_send(i2c_client, auc_i2c_write_buf, 2);
		mdelay(5);
	}while(i_ret <= 0 && i < 5 );
	printk("[FT520X] Step 2: Enter upgrade mode\n");
	
    /*********Step 3:check READ-ID***********************/        
   	cmd_write(0x90,0x00,0x00,0x00,4);
	i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
	i2c_master_recv(i2c_client, &reg_val, 2);
	printk("[FT520X] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	//byte_read(reg_val,2);
	if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
	{
		printk("[FT520X] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	}
	else
	{
		return 2;
		//i_is_new_protocol = 1;
	}

    //cmd_write(0xcd,0x0,0x00,0x00,1);
    //byte_read(reg_val,1);
    //printk("bootloader version = 0x%x\n", reg_val[0]);

    /* --------- read current project setting  ---------- */
    //set read start address
    buf[0] = 0x3;
    buf[1] = 0x0;
    buf[2] = 0x78;
    buf[3] = 0x0;
  //  i2c_master_send(i2c_client, &buf[0], 4);
    cmd_write(0x3,0x0,0x78,0x0,4);
    i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
    i2c_master_recv(i2c_client, &buf, 8);

    printk("[FTS] old setting: uc_i2c_addr = 0x%x, uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
        buf[0],  buf[2], buf[4]);
   
    /********* reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    msleep(200);

    return buf[4];
    
}

static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {             
	int err = 0,i = 0,uc_host_fm_ver;
//	uint8_t Firmware_version[3] = {0x20,0x00,0x00};
	i2c_client = client;
	unsigned char bufff[8] = {0},buff[8] = {0}; 
	
	printk(KERN_ERR"**********ft5306 %s**********\n", __FUNCTION__);

	#ifdef TPD_HAVE_POWER_ON_OFF
	tpd_hw_enable();
	
	//for power on sequence
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	
	#endif

	mdelay(200);

	do{
			uc_tp_factory_ID = fts_ctpm_get_panel_factory_setting();
			if (uc_tp_factory_ID == 0x53 || uc_tp_factory_ID == 0x5a)
					break;
			i++;
		printk("Failed to read factory_ID %d time(s)\n", i);
	}while(i < 5);
	
	if (uc_tp_factory_ID == 0x53){
		virtual_key[0][0] = 0;
		virtual_key[0][1] = 60;
		virtual_key[1][0] = 100;
		virtual_key[1][1] = 140;
		virtual_key[2][0] = 180;
		virtual_key[2][1] = 220;
		virtual_key[3][0] = 260;
		virtual_key[3][1] = 300;
	}
	
//	read_reg(0xa8, buff);		//hardware ID(truly or mudong)
	read_reg(0xa6, bufff);		//panel factory ID
	printk("############factory_ID = %2x, bufff = %2x ################\n",uc_tp_factory_ID,bufff[0]);
	printk("xxxxx%s's firmware version is 0x%2x xxxxxxxxxxxx\n", uc_tp_factory_ID==0x5a?version[0] : version[1], bufff[0]);

	#if TP_UPGRADE
	
		if (uc_tp_factory_ID == 0x5a)
		{
			uc_host_fm_ver = fts_ctpm_get_upg_ver(CTPM_TRULY_FW, sizeof(CTPM_TRULY_FW));
			//if(fts_ctpm_get_upg_ver(CTPM_TRULY_FW, sizeof(CTPM_TRULY_FW)) != bufff[0])
			if ((bufff[0] == 0xa6) || (bufff[0] < uc_host_fm_ver))
			{
				printk("Different version, we need version 0x%2x, while now is 0x%2x\n", 
								fts_ctpm_get_upg_ver(CTPM_TRULY_FW, sizeof(CTPM_TRULY_FW)), bufff[0]);
				fts_ctpm_fw_upgrade_with_i_file(CTPM_TRULY_FW, sizeof(CTPM_TRULY_FW));
				if(read_reg(0xa6,bufff)<0){
				printk(KERN_ERR"XXXXXXXXXXtp upgrade error!!\nXXXXXXXXXXX");
				}
				printk("@@@@@@@@@@after upgrade : bufff= %2x \n", bufff[0]);
			}
		}
		else if (uc_tp_factory_ID == 0x53)
		{
			uc_host_fm_ver = fts_ctpm_get_upg_ver(CTPM_MUDONG_FW, sizeof(CTPM_MUDONG_FW));
			//if(fts_ctpm_get_upg_ver(CTPM_MUDONG_FW, sizeof(CTPM_MUDONG_FW)) != bufff[0])
			if ((bufff[0] == 0xa6) || (bufff[0] < uc_host_fm_ver))
			{
				printk("Different version, we need version 0x%2x, while now is 0x%2x\n", 
								fts_ctpm_get_upg_ver(CTPM_MUDONG_FW, sizeof(CTPM_MUDONG_FW)), bufff[0]);
				fts_ctpm_fw_upgrade_with_i_file(CTPM_MUDONG_FW, sizeof(CTPM_MUDONG_FW));
				if(read_reg(0xa6,bufff)<0){
				printk(KERN_ERR"XXXXXXXXXXtp upgrade error!!\nXXXXXXXXXXX");
				}
				printk("@@@@@@@@@@after upgrade : bufff= %2x \n", bufff[0]);
			}
			
		}
		printk("xxxxx%s's firmware version is 0x%2x xxxxxxxxxxxx\n", uc_tp_factory_ID==0x5a?version[0] : version[1], bufff[0]);
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

    TPD_DEBUG_PRINT_INT; tpd_flag=1; wake_up_interruptible(&waiter);
} 
static int tpd_i2c_remove(struct i2c_client *client) {return 0;}

ssize_t mt6573_dma_read_m_byte(unsigned char cmd, unsigned char*returnData_va, u32 returnData_pa,unsigned char len)
{
    char     readData = 0;
    int     ret=0, read_length = 0;
    int    i, total_count = len;

    i2c_client->addr = (i2c_client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
    returnData_va[0] = cmd;//use as buffer
    ret = i2c_master_send(i2c_client, returnData_pa, 1);

    if (ret < 0) {
        printk(KERN_ERR"xxxx focal sends command error!! xxxx\n");
        return 0;
    }

    if (len > 0){
        ret = i2c_master_recv(i2c_client, returnData_pa, len);
        if (ret < 0) {
            printk(KERN_ERR"xxxx focal reads data error!! xxxx\n");
            return 0;
        }
    }
	i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
	
    return 1;
}


/*Get the virtual key type and report*/
static void report_key(struct tpd_data *data, u16 x, int flag)
{
	int key_type = 0; 
	input_report_abs(data->input, ABS_PRESSURE, 128*flag);
	input_report_key(data->input, BTN_TOUCH, flag);
	input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, 128*flag);
	if (x > virtual_key[0][0] && x < virtual_key[0][1]) {
		key_type = KEY_MENU;
		if(g_boot_mode == RECOVERY_BOOT)
		input_report_key(data->input, key_type, flag);
		else
		{
		input_report_abs(data->input, ABS_MT_POSITION_X, tpd_keys_dim_local[0][0]*flag);
		input_report_abs(data->input, ABS_MT_POSITION_Y, tpd_keys_dim_local[0][1]*flag);}
	} else if (x > virtual_key[1][0] && x < virtual_key[1][1]) {
		key_type = KEY_HOME;
		if(g_boot_mode == RECOVERY_BOOT)
		input_report_key(data->input, key_type, flag);
		else{
		input_report_abs(data->input, ABS_MT_POSITION_X, tpd_keys_dim_local[3][0]*flag);
		input_report_abs(data->input, ABS_MT_POSITION_Y, tpd_keys_dim_local[3][1]*flag);}
		} 
		 else if (x > virtual_key[2][0] && x < virtual_key[2][1]) {
		key_type = KEY_SEARCH;
		if(g_boot_mode == RECOVERY_BOOT)
		input_report_key(data->input, key_type, flag);
		else{
		input_report_abs(data->input, ABS_MT_POSITION_X, tpd_keys_dim_local[1][0]*flag);
		input_report_abs(data->input, ABS_MT_POSITION_Y, tpd_keys_dim_local[1][1]*flag);}
	}
		 else if (x > virtual_key[3][0] && x < virtual_key[3][1]) {
		key_type = KEY_BACK;
		if(g_boot_mode == RECOVERY_BOOT)
		input_report_key(data->input, key_type, flag);
		else{
		input_report_abs(data->input, ABS_MT_POSITION_X, tpd_keys_dim_local[2][0]*flag);
		input_report_abs(data->input, ABS_MT_POSITION_Y, tpd_keys_dim_local[2][1]*flag);}
	}
		 input_mt_sync(data->input);
//printk("xxxxxxxxxxxxx key_type = %d  xxxxxxxxxxxx\n", key_type);
//	if (key_type)
//		input_report_key(data->input, key_type, flag);
}

static int touch_event_handler(void *unused) {
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	int m =1;
	int index;
	uint8_t Firmware_version[3] = {0x20,0x00,0x00};
	struct tpd_data *data;
	int i, j = 3;

	data = kzalloc(sizeof(struct tpd_data), GFP_KERNEL);
	if (!data) {
		pr_err("%s:get memery failed!\n", __func__);
		return -ENOMEM;
	}

	data->input = tpd->dev;

	sched_setscheduler(current, SCHED_RR, &param);

	do {
	set_current_state(TASK_INTERRUPTIBLE);
	if (!kthread_should_stop()) {
            TPD_DEBUG_CHECK_NO_RESPONSE;
            do {
				while (tpd_halt) {tpd_flag = 0; /*sinfo.TouchpointFlag=0; */ msleep(20);}
               		wait_event_interruptible(waiter,tpd_flag!=0);
					tpd_flag = 0;
            } while(0);

            TPD_DEBUG_SET_TIME;
        }
       		 set_current_state(TASK_RUNNING);


	data->raw_data = tpDMABuf_va;
	mt6573_dma_read_m_byte(RAW_DATA_BEGIN_ADDR, tpDMABuf_va, tpDMABuf_pa, 13);

	
    data->total_tp = data->raw_data[2] & 0x0F;      /*Get current total pressed points*/
    if(data->total_tp > MAX_TOUCH_POINTS)           /*Ignore points which exceed the max pressed points we support */
		data->total_tp = MAX_TOUCH_POINTS;

	if (data->last_total_tp < data->total_tp) {	/*Had a new points pressed down*/
		data->last_total_tp = data->total_tp;
	}

	/*Transform data*/
	for(i = 0; i < data->last_total_tp; i++) {
		
		data->points[i].touch_event = (data->raw_data[3 + i * 6] & 0xC0) >> 6;
		data->points[i].touch_id = (data->raw_data[5 + i * 6] & 0xF0) >> 4;
		data->points[i].x = ((data->raw_data[3 + i * 6] & 0x0F) << 8) | data->raw_data[4 + i * 6];
		data->points[i].y = ((data->raw_data[5 + i * 6] & 0x0F) << 8) | data->raw_data[6 + i * 6];
	}
	
	
	for(i = 0; i < data->last_total_tp; i++) {
		if (data->points[i].y > KEY_Y_TOP) {	/*pressed on the key area*/
			if (data->points[i].touch_event == TOUCH_DOWN)
				report_key(data, data->points[i].x, EVENT_DOWN);
			else if (data->points[i].touch_event == TOUCH_UP)
				report_key(data, data->points[i].x, EVENT_UP);
		}else{
		
			if (data->points[i].touch_event == TOUCH_DOWN || data->points[i].touch_event == HOLD)
				data->points[i].pressed = EVENT_DOWN;
			else if(data->points[i].touch_event == TOUCH_UP)
					data->points[i].pressed = EVENT_UP;

			REPORT_TP(data, data->points[i]);

#if DEBUG
			printk("\n");
			printk("ABS_MT_TRACKING_ID: %d\n", data->points[i].touch_id);
			printk("ABS_MT_TRACKING_MAJOR: %d\n", data->points[i].pressed);
			printk("ABS_MT_POSITION_X: %d\n", data->points[i].x);
			printk("ABS_MT_POSITION_Y: %d\n", data->points[i].y);
#endif
		}
	}
	input_sync(data->input);

	data->last_total_tp = data->total_tp;	/*Record previous total pressed points*/
	continue;
    } while (!kthread_should_stop());
    return 0;
}


int tpd_local_init(void)
{

    boot_mode = get_boot_mode();
    // Software reset mode will be treated as normal boot
    if(boot_mode==3) boot_mode = NORMAL_BOOT;

//add wangdongliang DMA
    tpDMABuf_va = (unsigned char *)dma_alloc_coherent(NULL, 4096, &tpDMABuf_pa, GFP_KERNEL);
    if(!tpDMABuf_va){
		printk(KERN_ERR"xxxx Allocate DMA I2C Buffer failed!xxxx\n");
		return -1;
    }
//end wangdongliang DMA


     if(i2c_add_driver(&tpd_i2c_driver)!=0) {
      TPD_DMESG("unable to add i2c driver.\n");
      return -1;
    }

//button setting
//	set_bit(KEY_HOME, tpd->dev->keybit);
//	set_bit(KEY_MENU, tpd->dev->keybit);
//	set_bit(KEY_BACK, tpd->dev->keybit);
//	set_bit(KEY_SEARCH, tpd->dev->keybit);


//#ifdef TPD_HAVE_BUTTON
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
//#endif  

#if 0

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
#endif
    return 0;
}

/* Function to manage low power suspend */
void tpd_suspend(struct early_suspend *h)
{
	tpd_halt = 1;
   	 mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	 write_reg(0xa5, 3);

#ifdef TPD_HAVE_POWER_ON_OFF
	/*mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);*/
	//tpd_hw_disable();
#endif

}

/* Function to manage power-on resume */
void tpd_resume(struct early_suspend *h)
{
	printk("%s \n", __FUNCTION__);
#ifdef TPD_HAVE_POWER_ON_OFF
	//mt_set_gpio_mode(181, 1);
	//mt_set_gpio_mode(182, 1);
	/*mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);*/
//	tpd_hw_enable();
	mt_set_gpio_mode(GPIO43, 0);
	mt_set_gpio_dir(GPIO43, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO43, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO43, GPIO_OUT_ZERO);
	mdelay(5);
	mt_set_gpio_out(GPIO43, GPIO_OUT_ONE);
	mdelay(200);
#endif

	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	tpd_halt = 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "ft5306",
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
    printk(KERN_INFO"focaltec 5X02 touch panel driver init xxxx\n");


		if(tpd_driver_add(&tpd_device_driver) < 0)
			TPD_DMESG("add generic driver failed\n");
    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
    TPD_DMESG("MediaTek ft5306 touch panel driver exit\n");
    //input_unregister_device(tpd->dev);
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

