
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

#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#include "tpd.h"
#include <cust_eint.h>
#include <linux/jiffies.h>

#ifndef TPD_NO_GPIO 
#include "cust_gpio_usage.h"
#endif

extern struct tpd_device *tpd;

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

#define TPD_OK 0

#define TPD_CONFIG_REG_BASE           0x6A2
#define TPD_TOUCH_INFO_REG_BASE       0x712
#define TPD_POINT_INFO_REG_BASE       0x722
#define TPD_VERSION_INFO_REG          0x713
#define TPD_POWER_MODE_REG            0x692
#define TPD_HANDSHAKING_START_REG     0xFFF
#define TPD_HANDSHAKING_END_REG       0x8000
#define TPD_SOFT_RESET_MODE     0x01
#define TPD_POINT_INFO_LEN      8
#define TPD_MAX_POINTS          5
#define MAX_TRANSACTION_LENGTH 8
#define I2C_DEVICE_ADDRESS_LEN 2

#define TPD_X_RES 480
#define TPD_Y_RES 800

#define TPD_WARP_Y(y) ( TPD_Y_RES - 1 - y )
#define TPD_WARP_X(x)
 
#define MAX_I2C_TRANSFER_SIZE (MAX_TRANSACTION_LENGTH - I2C_DEVICE_ADDRESS_LEN)

#define GT818_CONFIG_PROC_FILE "gt818_config"
struct tpd_info_t
{
    u8 vendor_id_1;
    u8 vendor_id_2;
    u8 product_id_1;
    u8 product_id_2;
    u8 version_1;
    u8 version_2;
};

#if 0
//GT818A config
static u8 cfg_data[] =
{
    /* 6A2 */ 0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,
              0x10,0x12,0x60,0x00,0x50,0x00,0x40,0x00,
    /* 6B2 */ 0x30,0x00,0x20,0x00,0x10,0x00,0x00,0x00,
              0x70,0x00,0x80,0x00,0x90,0x00,0xA0,0x00,
    /* 6C2 */ 0xB0,0x00,0xC0,0x00,0xD0,0x00,0xE0,0x00,
              0xF0,0x00,0x01,0x13,0x80,0x88,0x90,0x14,
    /* 6D2 */ 0x15,0x40,0x0F,0x0F,0x0A,0x50,0x3C,0x0C,
              0x05,0x00,0x05,0xE0,0x01,0x20,0x03,0x00,
    /* 6E2 */ 0x00,0x46,0x5A,0x00,0x00,0x00,0x00,0x03,
              0x19,0x05,0x00,0x00,0x00,0x00,0x00,0x00,
    /* 6F2 */ 0x20,0x10,0x00,0x04,0x00,0x00,0x00,0x00,
              0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x38,
    /* 702 */ 0x00,0x3C,0x28,0x00,0x00,0x00,0x00,0x00,
              0x00,0x01
};
#endif

#if 0
// smooth
static u8 cfg_data[] =
{
    0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E,
    0x10, 0x12, 0x60, 0x00, 0x50, 0x00, 0x40, 0x00,
    0x30, 0x00, 0x20, 0x00, 0x10, 0x00, 0x00, 0x00,
    0x70, 0x00, 0x80, 0x00, 0x90, 0x00, 0xA0, 0x00,
    0xB0, 0x00, 0xC0, 0x00, 0xD0, 0x00, 0xE0, 0x00,
    0x90, 0x00, 0x11, 0x13, 0x80, 0x88, 0x90, 0x16,
    0x17, 0x40, 0x0F, 0x0F, 0x0A, 0x50, 0x3C, 0x48,
    0x03, 0x00, 0x05, 0xE0, 0x01, 0x20, 0x03, 0x00,
    0x00, 0x64, 0x5A, 0x40, 0x40, 0x00, 0x00, 0x03,
    0x19, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x20, 0x10, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x38,
    0x00, 0x3C, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01
};
#endif

#if 1
//no smooth
static u8 cfg_data[] =
{
    0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E,
    0x10, 0x12, 0x60, 0x00, 0x50, 0x00, 0x40, 0x00,
    0x30, 0x00, 0x20, 0x00, 0x10, 0x00, 0x00, 0x00,
    0x70, 0x00, 0x80, 0x00, 0x90, 0x00, 0xA0, 0x00,
    0xB0, 0x00, 0xC0, 0x00, 0xD0, 0x00, 0xE0, 0x00,
    0x00, 0x00, 0x11, 0x13, 0x90, 0x90, 0x90, 0x40,
    0x40, 0x40, 0x0F, 0x0F, 0x0A, 0x50, 0x3C, 0x49,
    0x03, 0x00, 0x05, 0xE0, 0x01, 0x20, 0x03, 0x00,
    0x00, 0x64, 0x64, 0x40, 0x40, 0x00, 0x00, 0x03,
    0x19, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x20, 0x10, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x38,
    0x00, 0x3C, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01
};
#endif

static struct i2c_client *i2c_client = NULL;
static const struct i2c_device_id tpd_i2c_id[] = {{"mtk-tpd",0},{}};
static unsigned short force[] = {0, 0xBA, I2C_CLIENT_END,I2C_CLIENT_END};
static const unsigned short * const forces[] = { force, NULL };
static struct i2c_client_address_data addr_data = { .forces = forces,};
static struct i2c_driver tpd_i2c_driver =
{                       
    .probe = tpd_i2c_probe,                                   
    .remove = tpd_i2c_remove,                           
    .detect = tpd_i2c_detect,                           
    .driver.name = "mtk-tpd", 
    .id_table = tpd_i2c_id,                             
    .address_data = &addr_data,                        
}; 


/* proc file system */
static int i2c_write_bytes( struct i2c_client *client, u16 addr, u8 *txbuf, int len );
static int i2c_write_dummy( struct i2c_client *client, u16 addr );
static struct proc_dir_entry *gt818_config_proc = NULL;

static int gt818_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    char *ptr = page;
    int i;

    ptr += sprintf( ptr, "==== GT818 config ====\n" );

    for ( i = 0 ; i < sizeof( cfg_data ) - 1 ; i++ )
    {
        ptr += sprintf( ptr, "0x%02X ", cfg_data[i] );

        if ( i%8 == 7 )
            ptr += sprintf( ptr, "\n" );
    }    

    ptr += sprintf( ptr, "\n" );
    *eof = 1;
    return ( ptr - page );
}

static int gt818_config_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
    printk("write count %ld\n", count );

    if ( count != ( sizeof( cfg_data ) - 1 ) )
    {
        printk("size not match [%d:%ld]\n", sizeof(cfg_data) - 1, count );
        return -EFAULT;
    }

    if (copy_from_user( cfg_data, buffer, count))
    {
        printk("copy from user fail\n");
        return -EFAULT;
    }
    
    i2c_write_dummy( i2c_client, TPD_HANDSHAKING_START_REG );

    i2c_write_bytes( i2c_client, TPD_CONFIG_REG_BASE, cfg_data, sizeof( cfg_data ) );

    i2c_write_dummy( i2c_client, TPD_HANDSHAKING_END_REG );
    return count;
}

static int i2c_read_bytes( struct i2c_client *client, u16 addr, u8 *rxbuf, int len )
{
    u8 buffer[I2C_DEVICE_ADDRESS_LEN];
    u8 retry;
    u16 left = len;
    u16 offset = 0;

    struct i2c_msg msg[2] =
    {
        {
            .addr = client->addr,
            .flags = 0,
            .buf = buffer,
            .len = I2C_DEVICE_ADDRESS_LEN,
            .timing = 400
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .timing = 400
        },
    };

    if ( rxbuf == NULL )
        return -1;

    TPD_DEBUG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len );

    while ( left > 0 )
    {
        buffer[0] = ( ( addr+offset ) >> 8 ) & 0xFF;
        buffer[1] = ( addr+offset ) & 0xFF;

        msg[1].buf = &rxbuf[offset];

        if ( left > MAX_TRANSACTION_LENGTH )
        {
            msg[1].len = MAX_TRANSACTION_LENGTH;
            left -= MAX_TRANSACTION_LENGTH;
            offset += MAX_TRANSACTION_LENGTH;
        }
        else
        {
            msg[1].len = left;
            left = 0;
        }

        retry = 0;

        while ( i2c_transfer( client->adapter, &msg[0], 2 ) != 2 )
        {
            retry++;

            if ( retry == 20 )
            {
                TPD_DEBUG("I2C read 0x%X length=%d failed\n", addr + offset, len);
                return -1;
            }
        }
    }

    return 0;
}

static int i2c_write_bytes( struct i2c_client *client, u16 addr, u8 *txbuf, int len )
{
    u8 buffer[MAX_TRANSACTION_LENGTH];
    u16 left = len;
    u16 offset = 0;
    u8 retry = 0;

    struct i2c_msg msg = 
    {
        .addr = client->addr,
        .flags = 0,
        .buf = buffer
    };


    if ( txbuf == NULL )
        return -1;

    TPD_DEBUG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len );

    while ( left > 0 )
    {
        retry = 0;

        buffer[0] = ( (addr+offset) >> 8 ) & 0xFF;
        buffer[1] = ( addr+offset ) & 0xFF;

        if ( left > MAX_I2C_TRANSFER_SIZE )
        {
            memcpy( &buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], MAX_I2C_TRANSFER_SIZE );
            msg.len = MAX_TRANSACTION_LENGTH;
            left -= MAX_I2C_TRANSFER_SIZE;
            offset += MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            memcpy( &buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], left );
            msg.len = left + I2C_DEVICE_ADDRESS_LEN;
            left = 0;
        }

        TPD_DEBUG("byte left %d offset %d\n", left, offset );

        while ( i2c_transfer( client->adapter, &msg, 1 ) != 1 )
        {
            retry++;

            if ( retry == 20 )
            {
                TPD_DEBUG("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                return -1;
            }
            else
                 TPD_DEBUG("I2C write retry %d addr 0x%X%X\n", retry, buffer[0], buffer[1]);

        }
    }

    return 0;
}

static int i2c_write_dummy( struct i2c_client *client, u16 addr )
{
    u8 buffer[MAX_TRANSACTION_LENGTH];

    struct i2c_msg msg =
    {
        .addr = client->addr,
        .flags = 0,
        .buf = buffer,
        .len = 2
    };

    TPD_DEBUG("i2c_write_dummy to device %02X address %04X\n", client->addr, addr );

    buffer[0] = (addr >> 8) & 0xFF;
    buffer[1] = (addr) & 0xFF;

    i2c_transfer( client->adapter, &msg, 1 ); 

    return 0;
}

static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, "mtk-tpd");
    return 0;
}

static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{             
    int err = 0;
    struct tpd_info_t tpd_info;

    i2c_client = client;
    
    // set deep sleep off
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);  
    msleep(10);  

    // power down CTP
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
    msleep(10);

    // power on CTP
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
    msleep(10);
    
    // set INT mode
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);
    //mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

    msleep(50);
        
    // Create proc file system
    gt818_config_proc = create_proc_entry( GT818_CONFIG_PROC_FILE, 0666, NULL);

    if ( gt818_config_proc == NULL )
    {
        printk("create_proc_entry %s failed\n", GT818_CONFIG_PROC_FILE );
    }
    else 
    {
        gt818_config_proc->read_proc = gt818_config_read_proc;
        gt818_config_proc->write_proc = gt818_config_write_proc;
    }

    //TODO: Get firmware version
    memset( &tpd_info, 0, sizeof( struct tpd_info_t ) );
    err = i2c_read_bytes( client, TPD_VERSION_INFO_REG, (u8 *)&tpd_info, sizeof( struct tpd_info_t ) );

    if ( err )
    {
        TPD_DMESG(TPD_DEVICE " fail to get tpd info %d\n", err );
        return err;
    }
    else
    {
        TPD_DMESG( "TPD info\n");
        TPD_DMESG( "vendor %02X %02X\n", tpd_info.vendor_id_1, tpd_info.vendor_id_2 );
        TPD_DMESG( "product %02X %02X\n", tpd_info.product_id_1, tpd_info.product_id_2 );
        TPD_DMESG( "version %02X %02X\n", tpd_info.version_1, tpd_info.version_2 );
    }

    //TODO: Load the init table
    // setting resolution, RES_X, RES_Y
    cfg_data[59] = TPD_RES_X&0xff;
    cfg_data[60] = (TPD_RES_X>>8)&0xff;
    cfg_data[61] = TPD_RES_Y&0xff;
    cfg_data[62] = (TPD_RES_Y>>8)&0xff;    
    err = i2c_write_bytes( client, TPD_CONFIG_REG_BASE, cfg_data, sizeof( cfg_data ) );

    if ( err )
    {
        TPD_DMESG(TPD_DEVICE " fail to write tpd cfg %d\n", err );
        return err;
    }

    if ( err )
    {
        TPD_DMESG(TPD_DEVICE " fail to write tpd power mode %d\n", err );
        return err;
    }


    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);

    if (IS_ERR(thread))
    { 
        err = PTR_ERR(thread);
        TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", err);
    }
 
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);

    mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
    //mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1);
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH, tpd_eint_interrupt_handler, 1);
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    tpd_load_status = 1;
    
    return 0;
}

static void tpd_eint_interrupt_handler(void)
{ 
    TPD_DEBUG_PRINT_INT;
    tpd_flag=1;
    wake_up_interruptible(&waiter);
} 
static int tpd_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static void tpd_down(int x, int y, int size)
{
    input_report_abs(tpd->dev, ABS_PRESSURE, 128);
    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 128);
    input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, 128);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);
    //TPD_DEBUG_PRINT_POINT( x, y, 1 );
    TPD_EM_PRINT(x, y, x, y, size, 1);
}

static void tpd_up(int x, int y)
{
    input_report_abs(tpd->dev, ABS_PRESSURE, 0);
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
    input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, 0);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);
    TPD_EM_PRINT(x, y, x, y, 0, 0);
    //TPD_DEBUG_PRINT_POINT( x, y, 0 );
}

static int touch_event_handler(void *unused)
{
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD }; 
    int x, y, id, size, finger_num = 0;
    static u8 buffer[ TPD_POINT_INFO_LEN*TPD_MAX_POINTS ];
    static char buf_status;
    static u8 id_mask = 0;
    u8 cur_mask;
    int idx;
    static int x_history[TPD_MAX_POINTS+1];
    static int y_history[TPD_MAX_POINTS+1];

    sched_setscheduler(current, SCHED_RR, &param); 

    do
    {
        set_current_state(TASK_INTERRUPTIBLE);

        while ( tpd_halt )
        {
            tpd_flag = 0;
            msleep(20);
        }

        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        TPD_DEBUG_SET_TIME;
        set_current_state(TASK_RUNNING); 
        
        i2c_write_dummy( i2c_client, TPD_HANDSHAKING_START_REG );

        i2c_read_bytes( i2c_client, TPD_TOUCH_INFO_REG_BASE, buffer, 1);
        TPD_DEBUG("[mtk-tpd] STATUS : %x\n", buffer[0]);
        
        finger_num = buffer[0] & 0x0f;
        buf_status = buffer[0] & 0xf0;
     
        if ( tpd == NULL || tpd->dev == NULL )
            continue;
        
        //printk("buf_status %02X finger %d\n", buf_status, finger_num );
        //TPD_DEBUG_PRINT_STATUS(buffer[0]);

        if ( finger_num )
        {
            i2c_read_bytes( i2c_client, TPD_POINT_INFO_REG_BASE, buffer, finger_num*TPD_POINT_INFO_LEN);
#if 0
            {
                u8 buf_tmp[ TPD_POINT_INFO_LEN*TPD_MAX_POINTS ];
                int loop = 0;

                for ( loop = 0 ; loop < 20 ; loop++ )
                {
                    i2c_read_bytes( i2c_client, TPD_POINT_INFO_REG_BASE, buf_tmp, finger_num*TPD_POINT_INFO_LEN);
                }
            }
#endif
        }
        
        cur_mask = 0;

        for ( idx = 0 ; idx < finger_num ; idx++ )
        {
            u8 *ptr = &buffer[ idx*TPD_POINT_INFO_LEN ];
            id = ptr[0];

            if ( id < TPD_MAX_POINTS+1 )
            {
                x = ptr[1] + (((int)ptr[2]) << 8);
                y = ptr[3] + (((int)ptr[4]) << 8);
                size = ptr[5] + (((int)ptr[6]) << 8);

                //if ( x == 0 )
                //    x = 1;

                tpd_down( x, TPD_WARP_Y(y), size );

                cur_mask |= ( 1 << id );
                x_history[id] = x;
                y_history[id] = y;
            }
            else
                TPD_DEBUG("Invalid id %d\n", id );
        }         
               
        if ( cur_mask != id_mask )
        {
            u8 diff = cur_mask^id_mask;
            idx = 0;

            while ( diff )
            {
                if ( ( ( diff & 0x01 ) == 1 ) &&
                     ( ( cur_mask >> idx ) & 0x01 ) == 0 )
                {
                    // check if key release
                    tpd_up( x_history[idx], TPD_WARP_Y(y_history[idx]) );                    
                }

                diff = ( diff >> 1 );
                idx++;
            }
            id_mask = cur_mask;
        }

        if ( tpd != NULL && tpd->dev != NULL )
            input_sync(tpd->dev);

        i2c_write_dummy( i2c_client, TPD_HANDSHAKING_END_REG );

    } while ( !kthread_should_stop() ); 

    return 0;
}

static int tpd_local_init(void) 
{

    if(i2c_add_driver(&tpd_i2c_driver)!=0)
    {
        TPD_DMESG("unable to add i2c driver.\n");
        return -1;
    }
    if(tpd_load_status == 0)
    {
    	TPD_DMESG("add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
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

/* Function to manage low power suspend */
//void tpd_suspend(struct i2c_client *client, pm_message_t message)
static int tpd_suspend( struct early_suspend *h )
{
    u8 mode = 0x01;
    i2c_write_dummy( i2c_client, TPD_HANDSHAKING_START_REG );
    i2c_write_bytes( i2c_client, TPD_POWER_MODE_REG, &mode, 1 );
    tpd_halt = 1;
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    //mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    printk("suspend!\n");
    return 0;
}

/* Function to manage power-on resume */
//void tpd_resume(struct i2c_client *client)
static int tpd_resume( struct early_suspend *h )
{
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
    //mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    i2c_write_dummy( i2c_client, TPD_HANDSHAKING_END_REG );
    tpd_halt = 0;
    printk("resume!\n");
    return 0;
}

static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = "gt818",
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
static int __init tpd_driver_init(void)
{
    TPD_DMESG("MediaTek gt818 touch panel driver init\n");

    if ( tpd_driver_add(&tpd_device_driver) < 0)
        TPD_DMESG("add generic driver failed\n");

    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    TPD_DMESG("MediaTek gt818 touch panel driver exit\n");
    //input_unregister_device(tpd->dev);
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

