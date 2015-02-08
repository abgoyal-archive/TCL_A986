

#ifndef __RTMP_PCI_H__
#define __RTMP_PCI_H__

#define RT28XX_HANDLE_DEV_ASSIGN(handle, dev_p)				\
	((struct os_cookie *)handle)->pci_dev = dev_p;

#ifdef LINUX
// set driver data
#define RT28XX_DRVDATA_SET(_a)			pci_set_drvdata(_a, net_dev);

#define RT28XX_PUT_DEVICE(dev_p)

#define SA_SHIRQ IRQF_SHARED

#ifdef PCI_MSI_SUPPORT
#define RTMP_MSI_ENABLE(_pAd) \
	{     struct os_cookie *_pObj = (struct os_cookie *)(_pAd->OS_Cookie); \
		(_pAd)->HaveMsi = pci_enable_msi(_pObj->pci_dev) == 0 ? TRUE : FALSE; \
	}

#define RTMP_MSI_DISABLE(_pAd) \
	{     struct os_cookie *_pObj = (struct os_cookie *)(_pAd->OS_Cookie); \
		if (_pAd->HaveMsi == TRUE) \
			pci_disable_msi(_pObj->pci_dev); \
		_pAd->HaveMsi = FALSE;  \
	}
#else
#define RTMP_MSI_ENABLE(_pAd)		do{}while(0)
#define RTMP_MSI_DISABLE(_pAd)		do{}while(0)
#endif // PCI_MSI_SUPPORT //

#define RTMP_PCI_DEV_UNMAP()										\
{	if (net_dev->base_addr)	{								\
		iounmap((void *)(net_dev->base_addr));				\
		release_mem_region(pci_resource_start(dev_p, 0),	\
							pci_resource_len(dev_p, 0)); }	\
	if (net_dev->irq) pci_release_regions(dev_p); }

#define PCI_REG_READ_WORD(pci_dev, offset, Configuration)   \
    if (pci_read_config_word(pci_dev, offset, &reg16) == 0)     \
        Configuration = le2cpu16(reg16);                        \
    else                                                        \
        Configuration = 0;

#define PCI_REG_WIRTE_WORD(pci_dev, offset, Configuration)  \
    reg16 = cpu2le16(Configuration);                        \
    pci_write_config_word(pci_dev, offset, reg16);

#endif // LINUX //

#endif // __RTMP_PCI_H__ //
