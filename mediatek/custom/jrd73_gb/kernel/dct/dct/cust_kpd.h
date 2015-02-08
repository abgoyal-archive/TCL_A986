
#ifndef _CUST_KPD_H_
#define _CUST_KPD_H_
#include <linux/input.h>
#include <cust_eint.h>

#define KPD_YES		1
#define KPD_NO		0

/* available keys (Linux keycodes) */
#define KEY_CALL	KEY_SEND
#define KEY_ENDCALL	KEY_END
#undef KEY_OK
#define KEY_OK		KEY_REPLY	/* DPAD_CENTER */
#define KEY_FOCUS	KEY_HP
#define KEY_AT		KEY_EMAIL
#define KEY_POUND	228	//KEY_KBDILLUMTOGGLE
#define KEY_STAR	227	//KEY_SWITCHVIDEOMODE
#define KEY_DEL 	KEY_BACKSPACE
#define KEY_SYM		KEY_COMPOSE
/* KEY_HOME */
/* KEY_BACK */
/* KEY_VOLUMEDOWN */
/* KEY_VOLUMEUP */
/* KEY_MUTE */
/* KEY_MENU */
/* KEY_UP */
/* KEY_DOWN */
/* KEY_LEFT */
/* KEY_RIGHT */
/* KEY_CAMERA */
/* KEY_POWER */
/* KEY_TAB */
/* KEY_ENTER */
/* KEY_LEFTSHIFT */
/* KEY_COMMA */
/* KEY_DOT */		/* PERIOD */
/* KEY_SLASH */
/* KEY_LEFTALT */
/* KEY_RIGHTALT */
/* KEY_SPACE */
/* KEY_SEARCH */
/* KEY_0 ~ KEY_9 */
/* KEY_A ~ KEY_Z */



#define KPD_KEY_DEBOUNCE  1024      /* (val / 32) ms */
#define KPD_PWRKEY_MAP    KEY_POWER

#define KPD_PWRKEY_USE_EINT       KPD_YES

/* HW keycode [0 ~ 71] -> Linux keycode */
#define KPD_INIT_KEYMAP()	\
{	\
	[1] = KEY_VOLUMEDOWN,		\
	[2] = KEY_VOLUMEUP,		\
	[10] = KEY_HOME,		\
	[11] = KEY_MENU,		\
	[19] = KEY_BACK,		\
	[20] = KEY_SEARCH,		\
}	 
/*****************************************************************/
/*******************Preload Customation***************************/
/*****************************************************************/
#define KPD_PWRKEY_EINT_GPIO  GPIO63

#define KPD_PWRKEY_GPIO_DIN  0

#define  KPD_DL_KEY1  8    /* KEY_POWER */
#define KPD_DL_KEY2  2    /* KEY_VOLUMEUP */
#define KPD_DL_KEY3  1    /* KEY_VOLUMEDOWN */
/*****************************************************************/
/*******************Uboot Customation***************************/
/*****************************************************************/
#define MT65XX_META_KEY  11    /* KEY_MENU */
#define MT65XX_RECOVERY_KEY  2    /* KEY_VOLUMEUP */
#define MT65XX_FACTORY_KEY  10    /* KEY_HOME */
/*****************************************************************/
/*******************factory Customation***************************/
/*****************************************************************/
#define KEYS_PWRKEY_MAP		{ KEY_POWER, "Power" }
#define DEFINE_KEYS_KEYMAP(x)		\
 struct key x[] = {	\
 	KEYS_PWRKEY_MAP,		\
{ KEY_VOLUMEDOWN,   "VLDown"  }, \
{ KEY_VOLUMEUP,   "VLUp"  }, \
{KEY_HOME,   "home"  }, \
{KEY_MENU,   "menu"  }, \
{KEY_BACK,   "back"  }, \
{KEY_SEARCH,   "search"  }, \
}
#define CUST_KEY_VOLUP  KEY_VOLUMEUP    
#define CUST_KEY_VOLDOWN  KEY_VOLUMEDOWN    
#define CUST_KEY_CENTER  KEY_HOME    
#define CUST_KEY_CONFIRM  KEY_HOME    
#define CUST_KEY_BACK  KEY_BACK    
/*****************************************************************/
/*******************recovery Customation****************************/
/*****************************************************************/
#define RECOVERY_KEY_VOLDOWN  KEY_VOLUMEDOWN   
#define RECOVERY_KEY_VOLUP  KEY_VOLUMEUP   
#define RECOVERY_KEY_CENTER  KEY_MENU   
#define RECOVERY_KEY_RIGHT  KEY_BACK   
#endif



