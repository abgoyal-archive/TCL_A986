
#ifndef _CUST_BAT_H_
#define _CUST_BAT_H_

typedef enum
{
	Cust_CC_800MA = 800,
	Cust_CC_700MA = 700,
	Cust_CC_650MA = 600,
	Cust_CC_550MA = 500,
	Cust_CC_450MA = 400,
	Cust_CC_400MA = 300,
	Cust_CC_200MA = 200,
	Cust_CC_70MA  = 100,
	Cust_CC_0MA	  = 0
}cust_charging_current_enum;

typedef struct{	
	unsigned int BattVolt;
	unsigned int BattPercent;
}VBAT_TO_PERCENT;

/* Battery Temperature Protection */
#define MAX_CHARGE_TEMPERATURE  43
#define MIN_CHARGE_TEMPERATURE  0
#define ERR_CHARGE_TEMPERATURE  0xFF

/* Recharging Battery Voltage */
#define RECHARGING_VOLTAGE      4110

/* Charging Current Setting */
#define CONFIG_USB_IF 						0   
#define USB_CHARGER_CURRENT_SUSPEND			Cust_CC_0MA		// def CONFIG_USB_IF
#define USB_CHARGER_CURRENT_UNCONFIGURED	Cust_CC_70MA	// def CONFIG_USB_IF
#define USB_CHARGER_CURRENT_CONFIGURED		Cust_CC_450MA	// def CONFIG_USB_IF
#define USB_CHARGER_CURRENT					Cust_CC_450MA
#define AC_CHARGER_CURRENT					Cust_CC_650MA	
/* Battery Meter Solution */
#define CONFIG_ADC_SOLUTION 	1

/* Battery Voltage and Percentage Mapping Table */
VBAT_TO_PERCENT Batt_VoltToPercent_Table[] = {
	/*BattVolt,BattPercent*/
	{3400,0},
	{3641,10},
	{3708,20},
	{3741,30},
	{3765,40},
	{3793,50},
	{3836,60},
	{3891,70},
	{3960,80},
	{4044,90},
	{4183,100},
};

/* Precise Tunning */
//#define BATTERY_AVERAGE_SIZE 	60
//#define BATTERY_AVERAGE_SIZE 	6
#define BATTERY_AVERAGE_SIZE 	30

/* Common setting */
#define R_CURRENT_SENSE 2				// 0.2 Ohm
#define R_BAT_SENSE 2					// times of voltage
#define R_I_SENSE 2						// times of voltage
#define R_CHARGER_1 270
#define R_CHARGER_2 39
#define R_CHARGER_SENSE ((R_CHARGER_1+R_CHARGER_2)/R_CHARGER_2)	// times of voltage
#define V_CHARGER_MAX 6000				// 6 V
#define V_CHARGER_MIN 4400				// 4.4 V
#define V_CHARGER_ENABLE 0				// 1:ON , 0:OFF

/* Teperature related setting */
#define RBAT_PULL_UP_R             24000
#define RBAT_PULL_UP_VOLT          2500
//#define TBAT_OVER_CRITICAL_LOW     68237
#define TBAT_OVER_CRITICAL_LOW     483954
#define BAT_TEMP_PROTECT_ENABLE    1
#define BAT_NTC_10 1
#define BAT_NTC_47 0

/* Battery Notify */
#define BATTERY_NOTIFY_CASE_0001
#define BATTERY_NOTIFY_CASE_0002
#define BATTERY_NOTIFY_CASE_0003
#define BATTERY_NOTIFY_CASE_0004
#define BATTERY_NOTIFY_CASE_0005

#endif /* _CUST_BAT_H_ */ 
