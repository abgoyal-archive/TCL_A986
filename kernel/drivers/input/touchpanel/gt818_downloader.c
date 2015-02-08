#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/fs.h> 
#include <linux/file.h> 
#include <linux/mm.h> 
#include <asm/uaccess.h> 
#include <linux/delay.h>
#include <mach/irqs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <cust_eint.h>
#include "cust_gpio_usage.h"
#include "tpd.h"

#define MTK_GT818_DOWNLOADER_NAME "gt818_downloader"

#define I2C_ADAPTER_ID 0
#define I2C_FAKE_CLIENT_ID 0x66
#define I2C_REAL_CLIENT_ID 0xA0
#define MAX_I2C_TRANSFER_SIZE 6

#define TPD_CONFIG_REG_BASE     0x6A2
#define TPD_TOUCH_INFO_REG_BASE 0x712
#define TPD_POINT_INFO_REG_BASE 0x722
#define TPD_VERSION_INFO_REG    0x713
#define TPD_POWER_MODE_REG      0x692
#define TPD_SOFT_RESET_MODE     0x01

#define  REG_WCHWK		0x1303
#define  BIT_STD_STD_PIN	0
#define  BIT_STD_I2C		1
#define  BIT_STD_GPIO0		2
#define  BIT_STD_GPIO_EDG	3
#define  REG_I2CSHDN	        0x1404
#define  REG_NVRCS		0x1201
#define  BIT_NVRAM_STROE	0
#define  BIT_NVRAM_RECALL	1
#define  BIT_NVRAM_LOCK		2
#define  REG_ANY                0x0000
#define  REG_ADDRESSED	        0x00FF

#define  NVRAM_LEN               0x0FF0   //	nvram total space
#define  NVRAM_BOOT_SECTOR_LEN	 0x0100	// boot sector 
#define  NVRAM_UPDATE_START_ADDR 0x4100
#define  I2C_FRAME_MAX_LENGTH    6   //IIC buffer length

#define GT818_SET_INT_PIN( level ) mt_set_gpio_out(GPIO_CTP_EINT_PIN, level) //null macro now

#define GT818_DOWNLOADER_DEBUG

#ifdef GT818_DOWNLOADER_DEBUG
    #define TPD_DOWNLOADER_DEBUG(a,arg...) printk("DWN" ": " a,##arg)
#else
    #define TPD_DOWNLOADER_DEBUG(arg...)
#endif

#define TPD_DOWNLOADER_LOG(a,arg...) printk("DWN" ": " a,##arg)

struct tpd_info_t
{
    u8 vendor_id_1;
    u8 vendor_id_2;
    u8 product_id_1;
    u8 product_id_2;
    u8 version_1;
    u8 version_2;
};

//const u8 i2c_addr[3]={0xBA,0xBA,0xBA};

static u8 inbuf[256];
static u8 outbuf[256];
static u8 guitar_i2c_address = 0xBA;
static int gt80x_update_rate;
static struct i2c_adapter *adapter;

static char *ctpfw;
module_param( ctpfw, charp, 0000);
MODULE_PARM_DESC( ctpfw, "CTP Firmware");

#define TPD_CONFIG_REG_BASE           0x6A2
#define TPD_VERSION_INFO_REG          0x713

static u8 cfg_data[] =
{
    0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E,
    0x10, 0x12, 0x60, 0x00, 0x50, 0x00, 0x40, 0x00,
    0x30, 0x00, 0x20, 0x00, 0x10, 0x00, 0x00, 0x00,
    0x70, 0x00, 0x80, 0x00, 0x90, 0x00, 0xA0, 0x00,
    0xB0, 0x00, 0xC0, 0x00, 0xD0, 0x00, 0xE0, 0x00,
    0x00, 0x00, 0x11, 0x13, 0x90, 0x90, 0x90, 0x40,
    0x40, 0x40, 0x0F, 0x0F, 0x0A, 0x50, 0x3C, 0x48,
    0x03, 0x00, 0x05, 0xE0, 0x01, 0x20, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x03,
    0x19, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x20, 0x10, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x38,
    0x00, 0x3C, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01
};

static int gt818_i2c_write( u8 device_id, u16 address, u8 *data, u16 len )
{
    u8 buf[256];
    u16 left = len;
    u16 offset = 0;
    u8 retry = 0;

    struct i2c_msg msg =
    {
        .addr = device_id,
        .flags = 0,
    };

    if ( data == NULL )
        return -1;

    msg.buf = (u8 *)buf;

    while ( left > 0 )
    {
        retry = 0;

        buf[0] = ( (address+offset) >> 8 )&0xFF;
        buf[1] = (address+offset)&0xFF;

        if ( left > MAX_I2C_TRANSFER_SIZE )
        {
            memcpy( &buf[2], &data[offset], MAX_I2C_TRANSFER_SIZE );
            msg.len = MAX_I2C_TRANSFER_SIZE + 2;
            left -= MAX_I2C_TRANSFER_SIZE;
            offset += MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            memcpy( &buf[2], &data[offset], left );
            msg.len = left + 2;
            left = 0;
        }
		
        while (i2c_transfer( adapter, &msg, 1 ) != 1 )
        {
            retry++;

            if ( retry == 20 )
            {
                TPD_DOWNLOADER_DEBUG("I2C write 0x%X%X length=%d failed\n", buf[0], buf[1], len);
                return 0;
            }
            //else
            //    TPD_DOWNLOADER_DEBUG("I2C write retry %d addr 0x%X%X\n", retry, buf[0], buf[1]);

            //return 0;
        }
		
    }
    return 1;	
}

static int gt818_i2c_read( u8 device_id, u16 address, u8 *data, u16 len )
{
    u8 buf[2];
    u8 retry;
    u16 left = len;
    u16 offset = 0;

    struct i2c_msg msg[2] =
    {
        {
            .addr = device_id,
            .flags = 0,
            .buf = buf,
            .len = 2,
        },
        {
            .addr = device_id,
            .flags = 0,
        },
    };


    if ( buf == NULL )
        return -1;

    while ( left > 0 )
    {
        buf[0] = ( (address+offset) >> 8 )&0xFF;
        buf[1] = (address+offset)&0xFF;

        msg[1].flags = I2C_M_RD;
        msg[1].buf = &data[offset];

        if ( left > 8 )
        {
            msg[1].len = 8;
            left -= 8;
            offset += 8;
        }
        else
        {
            msg[1].len = left;
            left = 0;
        }

        retry = 0;
       
        while (i2c_transfer( adapter, &msg[0], 2 ) != 2 )
        {
            retry++;

            if ( retry == 20 )
            {
                TPD_DOWNLOADER_DEBUG("I2C read 0x%X length=%d failed\n", address, len);
            }
            //return 0;
        }

    }
    return 1;
}

static u8 is_equal( u8 *src , u8 *dst , int len )
{
    int i;
    
    for( i = 0 ; i < len ; i++ )
    {
        TPD_DOWNLOADER_DEBUG("[%02X:%02X]\n", src[i], dst[i]);
    }

    for( i = 0 ; i < len ; i++ )
    {
        if ( src[i] != dst[i] )
        {
            return 0;
        }
    }
    
    return 1;
}

static u8 gt818_nvram_store( void )
{
    u8 ret;
    
    ret = gt818_i2c_read( guitar_i2c_address, REG_NVRCS, inbuf, 1 );
    
    if ( ret == 0 )
    {
        return 0;
    }
    
    if ( ( inbuf[0] & BIT_NVRAM_LOCK ) == BIT_NVRAM_LOCK )
    {
        return 0;
    }
    
    outbuf[0] = (1<<BIT_NVRAM_STROE);		//store command
	    
    ret = gt818_i2c_write( guitar_i2c_address, REG_NVRCS, outbuf, 1 );
    
    return ret;
}

static u8 gt818_nvram_recall( void )
{
    u8 ret;
    
    ret = gt818_i2c_read( guitar_i2c_address, REG_NVRCS, inbuf, 1 );
    
    if ( ret == 0 )
    {
        return 0;
    }
    
    if ( ( inbuf[0]&BIT_NVRAM_LOCK) == BIT_NVRAM_LOCK )
    {
        return 0;
    }
    
    outbuf[0] = ( 1 << BIT_NVRAM_RECALL );		//recall command
    ret = gt818_i2c_write( guitar_i2c_address , REG_NVRCS , outbuf, 1);
    return ret;
}

static u8 gt818_reset( void )
{
    u8 ret = 1;
    int i;
    
    outbuf[0] = 1;
    outbuf[1] = 1;

/*
    for ( i = 0 ; i < 3 ; i++ )
    {
        if ( gt818_i2c_write( i2c_addr[i], 0x1303, outbuf, 1 ) )
        {
            guitar_i2c_address = i2c_addr[i];
            TPD_DOWNLOADER_DEBUG("Detect address %0X\n", guitar_i2c_address );
            break;
        }
    }
*/

#if 0
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
    msleep(1000);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#else
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(20);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(20);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(500);
#endif

    //while ( gt818_i2c_write( guitar_i2c_address, 0x1303, outbuf, 1 ) == 0 );

search_i2c:

    while ( gt818_i2c_write( guitar_i2c_address, 0x00FF, outbuf, 0 ) == 0 );

    msleep( 50 );

    while ( gt818_i2c_read( guitar_i2c_address, 0x00FF, inbuf, 1 ) == 0 );

    if ( inbuf[0] != 0x55 )
    {
        printk("res %02X\n", inbuf[0] );
        goto search_i2c;
    }

    TPD_DOWNLOADER_DEBUG("Detect address %0X\n", guitar_i2c_address );
#if 0
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);		
    msleep(20);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(20);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
    msleep(500);
    return ret;	
}

static u8 gt818_set_address_2( u8 addr )
{
    u8 in;
    int i;

    for ( i = 0 ; i < 12 ; i++ )
    {
        if ( gt818_i2c_read( addr, REG_ANY, &in, 1 ) )
        {
            TPD_DOWNLOADER_DEBUG("Got response\n");
            return 1;
        }
        TPD_DOWNLOADER_DEBUG("wait for retry\n");
        msleep(50);
    } 
    return 0;
}

static u8 gt818_update_firmware( u8 *nvram, u16 length )
{
    u8 ret,err,retry_time;
    u16 cur_code_addr;
    u16 cur_frame_num, total_frame_num, cur_frame_len;
	
	
    if( length > NVRAM_LEN - NVRAM_BOOT_SECTOR_LEN )
    {
        TPD_DOWNLOADER_DEBUG("length too big %d %d\n", length, NVRAM_LEN - NVRAM_BOOT_SECTOR_LEN );
        return 0;
    }
    	
    total_frame_num = ( length + I2C_FRAME_MAX_LENGTH - 1) / I2C_FRAME_MAX_LENGTH;  

    //gt80x_update_sta = _UPDATING;
    gt80x_update_rate = 0;

    for( cur_frame_num = 0 ; cur_frame_num < total_frame_num ; cur_frame_num++ )	  
    {
        retry_time = 5;
        
        cur_code_addr = NVRAM_UPDATE_START_ADDR + cur_frame_num * I2C_FRAME_MAX_LENGTH; 	

        if( cur_frame_num == total_frame_num - 1 )
        {
            cur_frame_len = length - cur_frame_num * I2C_FRAME_MAX_LENGTH;
        }
        else
        {
            cur_frame_len = I2C_FRAME_MAX_LENGTH;
        }
        
        do
        {
            err = 0;

            ret = gt818_i2c_write( guitar_i2c_address, cur_code_addr, &nvram[cur_frame_num*I2C_FRAME_MAX_LENGTH], cur_frame_len );		

            if ( ret == 0 )
            {
                TPD_DOWNLOADER_DEBUG("write fail\n");
                err = 1;
            }

            ret = gt818_i2c_read( guitar_i2c_address, cur_code_addr, inbuf, cur_frame_len);
            
            if ( ret == 0 )
            {
                TPD_DOWNLOADER_DEBUG("read fail\n");
                err = 1;
            }
            
            if( is_equal( &nvram[cur_frame_num*I2C_FRAME_MAX_LENGTH], inbuf, cur_frame_len ) == 0 )
            {
                TPD_DOWNLOADER_DEBUG("not equal\n");
                err = 1;
            }
			
        } while ( err == 1 && (--retry_time) > 0 );
        
        if( err == 1 )
        {
            break;
        }
		
        gt80x_update_rate = ( cur_frame_num + 1 )*128/total_frame_num;
    
    }

    if( err == 1 )
    {
        TPD_DOWNLOADER_DEBUG("write nvram fail\n");
        return 0;
    }
    
    ret = gt818_nvram_store();
    
    msleep( 20 );

    if( ret == 0 )
    {
        TPD_DOWNLOADER_DEBUG("nvram store fail\n");
        return 0;
    }
    
    ret = gt818_nvram_recall();

    msleep( 20 );
    
    if( ret == 0 )
    {
        TPD_DOWNLOADER_DEBUG("nvram recall fail\n");
        return 0;
    }

    for ( cur_frame_num = 0 ; cur_frame_num < total_frame_num ; cur_frame_num++ )		 //	read out all the code
    {

        cur_code_addr = NVRAM_UPDATE_START_ADDR + cur_frame_num*I2C_FRAME_MAX_LENGTH;
        retry_time=5;

        if ( cur_frame_num == total_frame_num-1 )
        {
            cur_frame_len = length - cur_frame_num*I2C_FRAME_MAX_LENGTH;
        }
        else
        {
            cur_frame_len = I2C_FRAME_MAX_LENGTH;
        }
        
        do
        {
            err = 0;
            ret = gt818_i2c_read( guitar_i2c_address, cur_code_addr, inbuf, cur_frame_len);
            
            if ( ret == 0 )
            {
                err = 1;
            }
            
            if( is_equal( &nvram[cur_frame_num*I2C_FRAME_MAX_LENGTH], inbuf, cur_frame_len ) == 0 )
            {
                err = 1;
            }
        } while ( err == 1 && (--retry_time) > 0 );
        
        if( err == 1 )
        {
            break;
        }
        
        gt80x_update_rate = 127 + ( cur_frame_num + 1 )*128/total_frame_num;
    }
    
    gt80x_update_rate = 255;
    //gt80x_update_sta = _UPDATECHKCODE;

    if( err == 1 )
    {
        TPD_DOWNLOADER_DEBUG("nvram validate fail\n");
        return 0;
    }
    return 1;
}

static u8 gt818_update_proc( u8 *nvram, u16 length )
{
    u8 ret;
    u8 error = 0;
    struct tpd_info_t tpd_info;

    GT818_SET_INT_PIN( 0 );
    msleep( 20 );

    ret = gt818_reset();

    if ( ret == 0 )
    {
        error = 1;
        TPD_DOWNLOADER_DEBUG("reset fail\n");
        goto end;
    }
/*
    for( i = 0 ; i < 3 ; i++ )
    {
        if( guitar_i2c_address == i2c_addr[i] )
        {
            break;
        }

        if( i == 2 )
        {
            guitar_i2c_address = i2c_addr[0];
        }
    }
*/
    ret = gt818_set_address_2( guitar_i2c_address );

    if ( ret == 0 )
    {
        error = 1;
        TPD_DOWNLOADER_DEBUG("set address fail\n");
        goto end;
    }

    ret = gt818_update_firmware( nvram, length );

    if ( ret == 0 )
    {
        error=1;
        TPD_DOWNLOADER_DEBUG("firmware update fail\n");
        goto end;
    }

end:
    GT818_SET_INT_PIN( 1 );
    msleep( 1 );
    ret = gt818_reset();

    if ( ret == 0 )
    {
        error=1;
        TPD_DOWNLOADER_DEBUG("final reset fail\n");
        goto end;
    }
    if ( error == 1 )
    {
        return 0; 
    }

    while ( gt818_i2c_read( guitar_i2c_address, TPD_VERSION_INFO_REG, (u8 *)&tpd_info, sizeof( struct tpd_info_t ) ) == 0 ) ;

    TPD_DOWNLOADER_LOG( "version %02X %02X\n", tpd_info.version_1, tpd_info.version_2 );

    while ( gt818_i2c_write( guitar_i2c_address, TPD_CONFIG_REG_BASE, cfg_data, sizeof(cfg_data)) == 0 );

    return 1;
}
static int gt818_downloader_probe(struct i2c_client *client, const struct i2c_device_id *id )
{
    struct file *fp;
    u16 rlen;	
    u16 length;
    mm_segment_t old_fs;
    struct tpd_info_t tpd_info;
    u8 *nvram;
    u16 retry = 0;

    TPD_DOWNLOADER_LOG("gt818_downloader_probe\n");
    
    adapter = client->adapter;

    nvram = (u8 *)kmalloc( NVRAM_LEN, GFP_KERNEL );

    if ( nvram == NULL )
    {
        TPD_DOWNLOADER_LOG("Malloc fail\n");
        return -1;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    length = 0;

    if ( strlen( ctpfw ) > 0 )
    {
        fp = filp_open( ctpfw, O_RDONLY, 0 );
	
        if ( IS_ERR( fp ) )
        {
            TPD_DOWNLOADER_LOG("no file\n");
            fp = NULL;
        }
	
        if ( fp && fp->f_op && fp->f_op->read )
        {
            do
            {
                rlen = fp->f_op->read( fp, &nvram[length], 256, &fp->f_pos); 
                TPD_DOWNLOADER_DEBUG("read len %d\n", rlen);
                if ( rlen > 0 )
                {
                    length += rlen;
                }
                else
                   break;

            } while ( rlen > 0 );

            filp_close(fp,NULL); 
        }
        else
            TPD_DOWNLOADER_DEBUG("something wrong\n");
    }
    else
        TPD_DOWNLOADER_DEBUG("No firmware file loading\n");

    set_fs(old_fs);

    while ( gt818_update_proc( nvram, length ) == 0 && retry < 10 ) retry++;

    kfree( nvram );

    return 0;
}

static int gt818_downloader_remove( struct i2c_client *client )
{
    TPD_DOWNLOADER_LOG("gt818_downloader_remove\n");
    return 0;
}

static int gt818_downloader_detect( struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    TPD_DOWNLOADER_LOG("gt818_downloader_detect\n");
    strcpy( info->type, MTK_GT818_DOWNLOADER_NAME );
    return 0;

}

static const struct i2c_device_id gt818_downloader_ts_id[] = {
    { MTK_GT818_DOWNLOADER_NAME, 0 },
    { }
};

static unsigned short force_i2c[] = { 0, 0x08, I2C_CLIENT_END };

static const unsigned short *const forces[] = {
    force_i2c,
    NULL
};

static struct i2c_client_address_data addr_data = {
    .forces = forces,
};

static struct i2c_driver gt818_downloader = {
    .probe  = gt818_downloader_probe,
    .remove = gt818_downloader_remove,
    .detect = gt818_downloader_detect,
    .id_table = gt818_downloader_ts_id,
    .driver = {
            .name = MTK_GT818_DOWNLOADER_NAME,
    },
    .address_data = &addr_data,
};

int init_module(void)
{
    TPD_DOWNLOADER_LOG(KERN_INFO "init_module() called\n");
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_OUT);
    //mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    i2c_add_driver( &gt818_downloader );
    return 0;
}
 
void cleanup_module(void)
{
    TPD_DOWNLOADER_LOG(KERN_INFO "cleanup_module() called\n");
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    //mt_set_gpio_out(GPIO_CTP_EINT_PIN, GPIO_OUT_ZERO);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
    i2c_del_driver( &gt818_downloader );
}

MODULE_LICENSE("GPL");
