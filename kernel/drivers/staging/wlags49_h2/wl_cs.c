

#include <wl_version.h>

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/in.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/bitops.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <linux/ioport.h>

#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/cisreg.h>
#include <pcmcia/ciscode.h>
#include <pcmcia/ds.h>
#include <debug.h>

#include <hcf.h>
#include <dhf.h>
#include <hcfdef.h>

#include <wl_if.h>
#include <wl_internal.h>
#include <wl_util.h>
#include <wl_main.h>
#include <wl_netdev.h>
#include <wl_cs.h>
#include <wl_sysfs.h>


#if DBG
extern dbg_info_t *DbgInfo;
#endif  /* DBG */


static int wl_adapter_attach(struct pcmcia_device *link)
{
    struct net_device   *dev;
    struct wl_private	*lp;
    /*------------------------------------------------------------------------*/

    DBG_FUNC( "wl_adapter_attach" );
    DBG_ENTER( DbgInfo );

    dev = wl_device_alloc();
    if(dev == NULL) {
        DBG_ERROR( DbgInfo, "wl_device_alloc returned NULL\n");
	return -ENOMEM;
    }

    link->io.NumPorts1      = HCF_NUM_IO_PORTS;
    link->io.Attributes1    = IO_DATA_PATH_WIDTH_16;
    link->io.IOAddrLines    = 6;
    link->conf.Attributes   = CONF_ENABLE_IRQ;
    link->conf.IntType      = INT_MEMORY_AND_IO;
    link->conf.ConfigIndex  = 5;
    link->conf.Present      = PRESENT_OPTION;

    link->priv = dev;
    lp = wl_priv(dev);
    lp->link = link;

    wl_adapter_insert(link);

    DBG_LEAVE( DbgInfo );
    return 0;
} // wl_adapter_attach
/*============================================================================*/



static void wl_adapter_detach(struct pcmcia_device *link)
{
    struct net_device   *dev = link->priv;
    /*------------------------------------------------------------------------*/


    DBG_FUNC( "wl_adapter_detach" );
    DBG_ENTER( DbgInfo );
    DBG_PARAM( DbgInfo, "link", "0x%p", link );

    wl_adapter_release(link);

    if (dev) {
	unregister_wlags_sysfs(dev);
	unregister_netdev(dev);
    }

    wl_device_dealloc(dev);

    DBG_LEAVE( DbgInfo );
} // wl_adapter_detach
/*============================================================================*/


void wl_adapter_release( struct pcmcia_device *link )
{
    DBG_FUNC( "wl_adapter_release" );
    DBG_ENTER( DbgInfo );
    DBG_PARAM( DbgInfo, "link", "0x%p", link);

    /* Stop hardware */
    wl_remove(link->priv);

    pcmcia_disable_device(link);

    DBG_LEAVE( DbgInfo );
} // wl_adapter_release
/*============================================================================*/

static int wl_adapter_suspend(struct pcmcia_device *link)
{
    struct net_device *dev = link->priv;

    //if (link->open) {
	netif_device_detach(dev);
	wl_suspend(dev);
//// CHECK!            pcmcia_release_configuration(link->handle);
    //}

    return 0;
} // wl_adapter_suspend

static int wl_adapter_resume(struct pcmcia_device *link)
{
	struct net_device *dev = link->priv;

	wl_resume(dev);

	netif_device_attach( dev );

	return 0;
} // wl_adapter_resume

void wl_adapter_insert( struct pcmcia_device *link )
{
    struct net_device       *dev;
    int i;
    int                     ret;
    /*------------------------------------------------------------------------*/

    DBG_FUNC( "wl_adapter_insert" );
    DBG_ENTER( DbgInfo );
    DBG_PARAM( DbgInfo, "link", "0x%p", link );

    dev     = link->priv;

    /* Do we need to allocate an interrupt? */
    link->conf.Attributes |= CONF_ENABLE_IRQ;

    ret = pcmcia_request_io(link, &link->io);
    if (ret != 0)
        goto failed;

    ret = pcmcia_request_irq(link, (void *) wl_isr);
    if (ret != 0)
        goto failed;

    ret = pcmcia_request_configuration(link, &link->conf);
    if (ret != 0)
        goto failed;

    dev->irq        = link->irq;
    dev->base_addr  = link->io.BasePort1;

    SET_NETDEV_DEV(dev, &link->dev);
    if (register_netdev(dev) != 0) {
	printk("%s: register_netdev() failed\n", MODULE_NAME);
	goto failed;
    }

    register_wlags_sysfs(dev);

    printk(KERN_INFO "%s: Wireless, io_addr %#03lx, irq %d, ""mac_address ",
               dev->name, dev->base_addr, dev->irq);
    for( i = 0; i < ETH_ALEN; i++ ) {
        printk("%02X%c", dev->dev_addr[i], ((i < (ETH_ALEN-1)) ? ':' : '\n'));
    }

    DBG_LEAVE( DbgInfo );
    return;

failed:
    wl_adapter_release( link );

    DBG_LEAVE(DbgInfo);
    return;
} // wl_adapter_insert
/*============================================================================*/


int wl_adapter_open( struct net_device *dev )
{
    struct wl_private *lp = wl_priv(dev);
    struct pcmcia_device *link = lp->link;
    int         result = 0;
    int         hcf_status = HCF_SUCCESS;
    /*------------------------------------------------------------------------*/


    DBG_FUNC( "wl_adapter_open" );
    DBG_ENTER( DbgInfo );
	DBG_PRINT( "%s\n", VERSION_INFO );
    DBG_PARAM( DbgInfo, "dev", "%s (0x%p)", dev->name, dev );

    if(!pcmcia_dev_present(link))
    {
        DBG_LEAVE( DbgInfo );
        return -ENODEV;
    }

    link->open++;

    hcf_status = wl_open( dev );

    if( hcf_status != HCF_SUCCESS ) {
        link->open--;
        result = -ENODEV;
    }

    DBG_LEAVE( DbgInfo );
    return result;
} // wl_adapter_open
/*============================================================================*/


int wl_adapter_close( struct net_device *dev )
{
    struct wl_private *lp = wl_priv(dev);
    struct pcmcia_device *link = lp->link;
    /*------------------------------------------------------------------------*/


    DBG_FUNC( "wl_adapter_close" );
    DBG_ENTER( DbgInfo );
    DBG_PARAM( DbgInfo, "dev", "%s (0x%p)", dev->name, dev );

    if( link == NULL ) {
        DBG_LEAVE( DbgInfo );
        return -ENODEV;
    }

    DBG_TRACE( DbgInfo, "%s: Shutting down adapter.\n", dev->name );
    wl_close( dev );

    link->open--;

    DBG_LEAVE( DbgInfo );
    return 0;
} // wl_adapter_close
/*============================================================================*/

static struct pcmcia_device_id wl_adapter_ids[] = {
#if ! ((HCF_TYPE) & HCF_TYPE_HII5)
	PCMCIA_DEVICE_MANF_CARD(0x0156, 0x0003),
	PCMCIA_DEVICE_PROD_ID12("Agere Systems", "Wireless PC Card Model 0110",
			    0x33103a9b, 0xe175b0dd),
#else
	PCMCIA_DEVICE_MANF_CARD(0x0156, 0x0004),
	PCMCIA_DEVICE_PROD_ID12("Linksys", "WCF54G_Wireless-G_CompactFlash_Card",
                            0x0733cc81, 0x98a599e1),
#endif  // (HCF_TYPE) & HCF_TYPE_HII5
	PCMCIA_DEVICE_NULL,
	};
MODULE_DEVICE_TABLE(pcmcia, wl_adapter_ids);

static struct pcmcia_driver wlags49_driver = {
    .owner          = THIS_MODULE,
    .drv            = {
	.name   = DRIVER_NAME,
    },
    .probe	= wl_adapter_attach,
    .remove	= wl_adapter_detach,
    .id_table	= wl_adapter_ids,
    .suspend	= wl_adapter_suspend,
    .resume	= wl_adapter_resume,
};



int wl_adapter_init_module( void )
{
    int ret;
    /*------------------------------------------------------------------------*/


    DBG_FUNC( "wl_adapter_init_module" );
    DBG_ENTER( DbgInfo );
    DBG_TRACE( DbgInfo, "wl_adapter_init_module() -- PCMCIA\n" );

    ret = pcmcia_register_driver(&wlags49_driver);

    DBG_LEAVE( DbgInfo );
    return ret;
} // wl_adapter_init_module
/*============================================================================*/


void wl_adapter_cleanup_module( void )
{
    DBG_FUNC( "wl_adapter_cleanup_module" );
    DBG_ENTER( DbgInfo );
    DBG_TRACE( DbgInfo, "wl_adapter_cleanup_module() -- PCMCIA\n" );


    pcmcia_unregister_driver(&wlags49_driver);

    DBG_LEAVE( DbgInfo );
    return;
} // wl_adapter_cleanup_module
/*============================================================================*/


int wl_adapter_is_open( struct net_device *dev )
{
    struct wl_private *lp = wl_priv(dev);
    struct pcmcia_device *link = lp->link;

    if(!pcmcia_dev_present(link)) {
        return 0;
    }

    return( link->open );
} // wl_adapter_is_open
/*============================================================================*/


#if DBG

const char* DbgEvent( int mask )
{
    static char DbgBuffer[256];
    char *pBuf;
    /*------------------------------------------------------------------------*/


    pBuf    = DbgBuffer;
    *pBuf   = '\0';


    if( mask & CS_EVENT_WRITE_PROTECT )
        strcat( pBuf, "WRITE_PROTECT " );

    if(mask & CS_EVENT_CARD_LOCK)
        strcat( pBuf, "CARD_LOCK " );

    if(mask & CS_EVENT_CARD_INSERTION)
        strcat( pBuf, "CARD_INSERTION " );

    if(mask & CS_EVENT_CARD_REMOVAL)
        strcat( pBuf, "CARD_REMOVAL " );

    if(mask & CS_EVENT_BATTERY_DEAD)
        strcat( pBuf, "BATTERY_DEAD " );

    if(mask & CS_EVENT_BATTERY_LOW)
        strcat( pBuf, "BATTERY_LOW " );

    if(mask & CS_EVENT_READY_CHANGE)
        strcat( pBuf, "READY_CHANGE " );

    if(mask & CS_EVENT_CARD_DETECT)
        strcat( pBuf, "CARD_DETECT " );

    if(mask & CS_EVENT_RESET_REQUEST)
        strcat( pBuf, "RESET_REQUEST " );

    if(mask & CS_EVENT_RESET_PHYSICAL)
        strcat( pBuf, "RESET_PHYSICAL " );

    if(mask & CS_EVENT_CARD_RESET)
        strcat( pBuf, "CARD_RESET " );

    if(mask & CS_EVENT_REGISTRATION_COMPLETE)
        strcat( pBuf, "REGISTRATION_COMPLETE " );

    // if(mask & CS_EVENT_RESET_COMPLETE)
    //     strcat( pBuf, "RESET_COMPLETE " );

    if(mask & CS_EVENT_PM_SUSPEND)
        strcat( pBuf, "PM_SUSPEND " );

    if(mask & CS_EVENT_PM_RESUME)
        strcat( pBuf, "PM_RESUME " );

    if(mask & CS_EVENT_INSERTION_REQUEST)
        strcat( pBuf, "INSERTION_REQUEST " );

    if(mask & CS_EVENT_EJECTION_REQUEST)
        strcat( pBuf, "EJECTION_REQUEST " );

    if(mask & CS_EVENT_MTD_REQUEST)
        strcat( pBuf, "MTD_REQUEST " );

    if(mask & CS_EVENT_ERASE_COMPLETE)
        strcat( pBuf, "ERASE_COMPLETE " );

    if(mask & CS_EVENT_REQUEST_ATTENTION)
        strcat( pBuf, "REQUEST_ATTENTION " );

    if(mask & CS_EVENT_CB_DETECT)
        strcat( pBuf, "CB_DETECT " );

    if(mask & CS_EVENT_3VCARD)
        strcat( pBuf, "3VCARD " );

    if(mask & CS_EVENT_XVCARD)
        strcat( pBuf, "XVCARD " );


    if( *pBuf ) {
        pBuf[strlen(pBuf) - 1] = '\0';
    } else {
        if( mask != 0x0 ) {
            sprintf( pBuf, "<<0x%08x>>", mask );
        }
    }

    return pBuf;
} // DbgEvent
/*============================================================================*/

#endif  /* DBG */
