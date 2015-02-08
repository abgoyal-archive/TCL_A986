

#if defined(CONFIG_USB_STORAGE_ONETOUCH) || \
		defined(CONFIG_USB_STORAGE_ONETOUCH_MODULE)

UNUSUAL_DEV(  0x0d49, 0x7000, 0x0000, 0x9999,
		"Maxtor",
		"OneTouch External Harddrive",
		US_SC_DEVICE, US_PR_DEVICE, onetouch_connect_input,
		0),

UNUSUAL_DEV(  0x0d49, 0x7010, 0x0000, 0x9999,
		"Maxtor",
		"OneTouch External Harddrive",
		US_SC_DEVICE, US_PR_DEVICE, onetouch_connect_input,
		0),

#endif /* defined(CONFIG_USB_STORAGE_ONETOUCH) || ... */
