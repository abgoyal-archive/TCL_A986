
#ifndef _ANDROID_HW_LCD_H
#define _ANDROID_HW_LCD_H

#define  LCD_DENSITY_MIN      120
#define  LCD_DENSITY_DEFAULT  160
#define  LCD_DENSITY_MAX      240

/* Sets the boot property corresponding to the emulated abstract LCD density */
extern void  hwLcd_setBootProperty(int density);

#endif /* _ANDROID_HW_LCD_H */

