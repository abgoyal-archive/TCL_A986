

#if defined(CONFIG_USB_STORAGE_KARMA) || \
		defined(CONFIG_USB_STORAGE_KARMA_MODULE)

UNUSUAL_DEV(  0x045a, 0x5210, 0x0101, 0x0101,
		"Rio",
		"Rio Karma",
		US_SC_SCSI, US_PR_KARMA, rio_karma_init, 0),

#endif /* defined(CONFIG_USB_STORAGE_KARMA) || ... */
