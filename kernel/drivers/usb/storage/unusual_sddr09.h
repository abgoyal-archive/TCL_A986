

#if defined(CONFIG_USB_STORAGE_SDDR09) || \
		defined(CONFIG_USB_STORAGE_SDDR09_MODULE)

UNUSUAL_DEV(  0x0436, 0x0005, 0x0100, 0x0100,
		"Microtech",
		"CameraMate (DPCM_USB)",
		US_SC_SCSI, US_PR_DPCM_USB, NULL, 0),

UNUSUAL_DEV(  0x04e6, 0x0003, 0x0000, 0x9999,
		"Sandisk",
		"ImageMate SDDR09",
		US_SC_SCSI, US_PR_EUSB_SDDR09, usb_stor_sddr09_init,
		0),

/* This entry is from Andries.Brouwer@cwi.nl */
UNUSUAL_DEV(  0x04e6, 0x0005, 0x0100, 0x0208,
		"SCM Microsystems",
		"eUSB SmartMedia / CompactFlash Adapter",
		US_SC_SCSI, US_PR_DPCM_USB, usb_stor_sddr09_dpcm_init,
		0),

UNUSUAL_DEV(  0x066b, 0x0105, 0x0100, 0x0100,
		"Olympus",
		"Camedia MAUSB-2",
		US_SC_SCSI, US_PR_EUSB_SDDR09, usb_stor_sddr09_init,
		0),

UNUSUAL_DEV(  0x0781, 0x0200, 0x0000, 0x9999,
		"Sandisk",
		"ImageMate SDDR-09",
		US_SC_SCSI, US_PR_EUSB_SDDR09, usb_stor_sddr09_init,
		0),

UNUSUAL_DEV(  0x07af, 0x0006, 0x0100, 0x0100,
		"Microtech",
		"CameraMate (DPCM_USB)",
		US_SC_SCSI, US_PR_DPCM_USB, NULL, 0),

#endif /* defined(CONFIG_USB_STORAGE_SDDR09) || ... */
