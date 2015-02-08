
/* system header files */
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/mtd/nand.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/system.h>
#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/pmu6573_sw.h>


#include <mach/mt6573_gpio.h>
#include <mach/mt6573_eint.h>
#include <mach/mt_bt.h>
#include <mach/mtk_rtc.h>

#include <cust_gpio_usage.h>
#include <cust_eint.h>


#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE)
#include <mach/mt_combo.h>
static void combo_bt_pcm_pin_on(void);
static void combo_bt_pcm_pin_off(void);
static void combo_fm_i2s_pin_on(void);
static void combo_fm_i2s_pin_off(void);
#endif
#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE)
    /* combo chip */
    #if defined(MTK_MT6620)
void mt6620_power_on(void);
void mt6620_power_off(void);
void mt6620_print_pin_configure(void);
    #endif
#endif

#if defined(CONFIG_MTK_COMBO_SDIO_SLOT)
static sdio_irq_handler_t combo_sdio_eirq_handler = NULL;
static pm_callback_t combo_sdio_pm_cb = NULL;
static void *combo_sdio_pm_data = NULL;
static void *combo_sdio_eirq_data = NULL;
//static pm_message_t mt_wifi_pm_state = { .event = PM_EVENT_HIBERNATE };
//static int mt_wifi_pm_late_cb = 0;

    #if (CONFIG_MTK_COMBO_SDIO_SLOT == 1)
const static u32 combo_sdio_eint_pin = GPIO_WIFI_EINT_PIN;
const static u32 combo_sdio_eint_num = CUST_EINT_WIFI_NUM;
const static u32 combo_sdio_eint_m_eint = GPIO_WIFI_EINT_PIN_M_EINT;
const static u32 combo_sdio_eint_m_gpio = GPIO_WIFI_EINT_PIN_M_GPIO;
static unsigned char combo_port_pwr_map[3] = {0xFF, 0x0, 0xFF};

    #else
    #error "unsupported CONFIG_MTK_COMBO_SDIO_SLOT" CONFIG_MTK_COMBO_SDIO_SLOT
    #endif

#else
static sdio_irq_handler_t mt_wifi_irq_handler = NULL;
static pm_message_t mt_wifi_pm_state = { .event = PM_EVENT_HIBERNATE };
static pm_callback_t mt_wifi_pm_cb = NULL;
static void *mt_wifi_pm_data = NULL;
static void *mt_wifi_irq_data = NULL;
static int mt_wifi_pm_late_cb = 0;
#endif

/*=======================================================================*/
/* Board Specific Devices Power Management                               */
/*=======================================================================*/
extern kal_bool upmu_is_chr_det(upmu_chr_list_enum chr);

void mt6573_power_off(void)
{
	printk("mt6573_power_off\n");

	rtc_bbpu_power_down();

	if (upmu_is_chr_det(CHR) == KAL_TRUE)
		arch_reset(0, "charger");

	while (1);
}

/*=======================================================================*/
/* Board Specific Devices                                                */
/*=======================================================================*/
/*GPS driver*/
/*FIXME: remove mt3326 notation */
struct mt3326_gps_hardware mt3326_gps_hw = {
    .ext_power_on =  NULL,
    .ext_power_off = NULL,
};

/*=======================================================================*/
/* Board Specific Devices Init                                           */
/*=======================================================================*/
void mt_bt_power_on(void)
{
    printk(KERN_INFO "+mt_bt_power_on\n");

#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE)
    /* combo chip product */
    #if defined(MTK_MT6620)
    /*
     * Ignore rfkill0/state call. Controll BT power on/off through device /dev/stpbt.
     */
    #else
    printk(KERN_WARNING "no bt device to power on!\n");
    #endif /* end of defined(MTK_MT6620) */
#endif
    printk(KERN_INFO "-mt_bt_power_on\n");
}
EXPORT_SYMBOL(mt_bt_power_on);

void mt_bt_power_off(void)
{
    printk(KERN_INFO "+mt_bt_power_off\n");

#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE)
    /* combo chip product */
    #if defined(MTK_MT6620)
    /*
     * Ignore rfkill0/state call. Controll BT power on/off through device /dev/stpbt.
     */
    #else
    printk(KERN_WARNING "no bt device to power off!\n");
    #endif /* end of defined(MTK_MT6620) */
#endif
    printk(KERN_INFO "-mt_bt_power_off\n");
}
EXPORT_SYMBOL(mt_bt_power_off);


int mt_bt_suspend(pm_message_t state)
{
    printk(KERN_INFO "+mt_bt_suspend\n");
    printk(KERN_INFO "-mt_bt_suspend\n");
    return MT_BT_OK;
}

int mt_bt_resume(pm_message_t state)
{
    printk(KERN_INFO "+mt_bt_resume\n");
    printk(KERN_INFO "-mt_bt_resume\n");
    return MT_BT_OK;
}

#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE)

static void combo_bgf_eirq_handler(void)
{
    mt_combo_bgf_eirq_handler(NULL);
}

static void mt_combo_bgf_request_irq(void *data)
{
    mt65xx_eint_set_sens(CUST_EINT_COMBO_BGF_NUM, CUST_EINT_COMBO_BGF_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_COMBO_BGF_NUM, CUST_EINT_COMBO_BGF_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_COMBO_BGF_NUM,
        CUST_EINT_COMBO_BGF_DEBOUNCE_EN,
        CUST_EINT_COMBO_BGF_POLARITY,
        combo_bgf_eirq_handler,
        0);
    mt65xx_eint_mask(CUST_EINT_COMBO_BGF_NUM); /*2*/
    return;
}

/* Combo chip shared interrupt (BGF_INT_B) */
void mt_combo_bgf_enable_irq(void)
{
    mt65xx_eint_unmask(CUST_EINT_COMBO_BGF_NUM);
    return;
}
EXPORT_SYMBOL(mt_combo_bgf_enable_irq);

void mt_combo_bgf_disable_irq(void)
{
    mt65xx_eint_mask(CUST_EINT_COMBO_BGF_NUM);
    return;
}
EXPORT_SYMBOL(mt_combo_bgf_disable_irq);

void mt6620_print_pin_configure(void)
{
	printk(KERN_INFO "[MT6620_PIN]=>GPIO pin configuration start<=\n");
#ifdef GPIO_COMBO_6620_LDO_EN_PIN
	printk(KERN_INFO "LDO_EN(GPIO%d)\n", GPIO_COMBO_6620_LDO_EN_PIN);
#else
	printk(KERN_INFO "LDO_EN(not defined)\n");	
#endif

#ifdef GPIO_COMBO_PMU_EN_PIN
	printk(KERN_INFO "PMU_EN(GPIO%d)\n", GPIO_COMBO_PMU_EN_PIN);
#else
	printk(KERN_INFO "PMU_EN(not defined)\n");	
#endif


#ifdef GPIO_COMBO_RST_PIN
	printk(KERN_INFO "RST(GPIO%d)\n", GPIO_COMBO_RST_PIN);
#else
	printk(KERN_INFO "RST(not defined)\n");	
#endif

#ifdef GPIO_COMBO_ALL_EINT_PIN
	printk(KERN_INFO "ALL_EINT(GPIO%d)\n", GPIO_COMBO_ALL_EINT_PIN);
#else
	printk(KERN_INFO "ALL_EINT(not defined)\n");
#endif


#ifdef GPIO_COMBO_BGF_EINT_PIN
	printk(KERN_INFO "BGF_EINT(GPIO%d)\n", GPIO_COMBO_BGF_EINT_PIN);
#else
	printk(KERN_INFO "BGF_EINT(not defined)\n");	
#endif

#ifdef CUST_EINT_COMBO_BGF_NUM
	printk(KERN_INFO "BGF_EINT_NUM(%d)\n", CUST_EINT_COMBO_BGF_NUM);
#else
	printk(KERN_INFO "BGF_EINT_NUM(not defined)\n");	
#endif

#ifdef GPIO_WIFI_EINT_PIN
	printk(KERN_INFO "WIFI_EINT(GPIO%d)\n", GPIO_WIFI_EINT_PIN);
#else
	printk(KERN_INFO "WIFI_EINT(not defined)\n");	
#endif

#ifdef CUST_EINT_WIFI_NUM
	printk(KERN_INFO "WIFI_EINT_NUM(%d)\n", CUST_EINT_WIFI_NUM);
#else
	printk(KERN_INFO "WIFI_EINT_NUM(not defined)\n");	
#endif

#ifdef GPIO_UART_URXD3_PIN
	printk(KERN_INFO "UART_RX(GPIO%d)\n", GPIO_UART_URXD3_PIN);
#else
	printk(KERN_INFO "UART_RX(not defined)\n");	
#endif
#ifdef GPIO_UART_UTXD3_PIN
	printk(KERN_INFO "UART_TX(GPIO%d)\n", GPIO_UART_UTXD3_PIN);
#else
	printk(KERN_INFO "UART_TX(not defined)\n");	
#endif
#ifdef GPIO_PCM_DAICLK_PIN
	printk(KERN_INFO "DAICLK(GPIO%d)\n", GPIO_PCM_DAICLK_PIN);
#else
	printk(KERN_INFO "DAICLK(not defined)\n");	
#endif
#ifdef GPIO_PCM_DAIPCMOUT_PIN
	printk(KERN_INFO "PCMOUT(GPIO%d)\n", GPIO_PCM_DAIPCMOUT_PIN);
#else
	printk(KERN_INFO "PCMOUT(not defined)\n");	
#endif
#ifdef GPIO_PCM_DAIPCMIN_PIN
	printk(KERN_INFO "PCMIN(GPIO%d)\n", GPIO_PCM_DAIPCMIN_PIN);
#else
	printk(KERN_INFO "PCMIN(not defined)\n");	
#endif
#ifdef GPIO_PCM_DAISYNC_PIN
	printk(KERN_INFO "PCMSYNC(GPIO%d)\n", GPIO_PCM_DAISYNC_PIN);
#else
	printk(KERN_INFO "PCMSYNC(not defined)\n");	
#endif
#ifndef FM_ANALOG_INPUT
	#ifdef GPIO_I2S1_CK_PIN
		printk(KERN_INFO "I2S_CK(GPIO%d)\n", GPIO_I2S1_CK_PIN);
	#else
		printk(KERN_INFO "I2S_CK(not defined)\n");	
	#endif
	#ifdef GPIO_I2S1_WS_PIN
		printk(KERN_INFO "I2S_WS(GPIO%d)\n", GPIO_I2S1_WS_PIN);
	#else
		printk(KERN_INFO "I2S_WS(not defined)\n");	
	#endif
	#ifdef GPIO_I2S1_DAT_PIN
		printk(KERN_INFO "I2S_DAT(GPIO%d)\n", GPIO_I2S1_DAT_PIN);
	#else
		printk(KERN_INFO "I2S_DAT(not defined)\n");	
	#endif
#else	//FM_ANALOG_INPUT
	printk(KERN_INFO "FM analog mode is set, no need for I2S GPIOs\n");	
#endif	//FM_ANALOG_INPUT
//	printk(KERN_INFO "[MT6620_PIN]GPIO25-->additional pmu_en v2.8 control, works only for re-worked phone\n");
	printk(KERN_INFO "[MT6620_PIN]=>GPIO pin configuration end<=\n");
	
}

void mt6620_power_on(void)
{
    int result = 0;
    static int _32k_set = 0;
	/*log MT6620 GPIO Settings*/
	mt6620_print_pin_configure();
    /* disable interrupt firstly */
    mt_combo_bgf_disable_irq();

#define MT6620_OFF_TIME (10) /* in ms, workable value */
#define MT6620_RST_TIME (30) /* in ms, workable value */
#define MT6620_STABLE_TIME (30) /* in ms, workable value */
#define MT6620_EXT_INT_TIME (5) /* in ms, workable value */
#define MT6620_32K_STABLE_TIME (100) /* in ms, test value */

    //printk(KERN_INFO "[mt6620] enable external LDO\n");
    /* disable pull */
    mt_set_gpio_pull_enable(GPIO_COMBO_6620_LDO_EN_PIN, GPIO_PULL_DISABLE);
    /* set output */
    mt_set_gpio_dir(GPIO_COMBO_6620_LDO_EN_PIN, GPIO_DIR_OUT);
    /* set gpio mode */
    mt_set_gpio_mode(GPIO_COMBO_6620_LDO_EN_PIN, GPIO_MODE_GPIO);
    /* external LDO_EN high */
    mt_set_gpio_out(GPIO_COMBO_6620_LDO_EN_PIN, GPIO_OUT_ONE);

#if defined(CONFIG_MTK_COMBO_SDIO_SLOT) && (CONFIG_MTK_COMBO_SDIO_SLOT == 1)
    //printk(KERN_INFO "[mt6620] pull up sd1 bus(gpio62~68)\n");
    mt_set_gpio_pull_enable(GPIO62, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO62, GPIO_PULL_UP);
    mt_set_gpio_pull_enable(GPIO63, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO63, GPIO_PULL_UP);
    mt_set_gpio_pull_enable(GPIO64, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO64, GPIO_PULL_UP);
    mt_set_gpio_pull_enable(GPIO65, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO65, GPIO_PULL_UP);
    mt_set_gpio_pull_enable(GPIO66, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO66, GPIO_PULL_UP);
    mt_set_gpio_pull_enable(GPIO67, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO67, GPIO_PULL_UP);
    mt_set_gpio_pull_enable(GPIO68, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO68, GPIO_PULL_UP);
#endif

    //printk(KERN_INFO "[mt6620] enable RTC GPIO\n");
    if(_32k_set == 0){
        //rtc_gpio_export_32k(true); //old 32k export API
        /*
         * To fix RTC32k clocks stops after system reboot
         */
        rtc_gpio_enable_32k(RTC_GPIO_USER_GPS);
        _32k_set = 1;
        printk("[mt6620]rtc_gpio_enable_32k(RTC_GPIO_USER_GPS) \n");
    } else {
        printk("[mt6620]not to rtc_gpio_enable_32k(RTC_GPIO_USER_GPS)\n");
    }

    msleep(MT6620_32K_STABLE_TIME);

    /* UART Mode */
    result += mt_set_gpio_mode(GPIO_UART_URXD3_PIN, GPIO_MODE_01);
    result += mt_set_gpio_mode(GPIO_UART_UTXD3_PIN, GPIO_MODE_01);
    //printk(KERN_INFO "[mt6620] set UART GPIO Mode [%d]\n", result);

    /* FIXME! GeorgeKuo: added for MT6620 GPIO initialization */
    /* disable pull */
    mt_set_gpio_pull_enable(GPIO_COMBO_PMU_EN_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_pull_enable(GPIO_COMBO_RST_PIN, GPIO_PULL_DISABLE);
    /* set output */
    mt_set_gpio_dir(GPIO_COMBO_PMU_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_dir(GPIO_COMBO_RST_PIN, GPIO_DIR_OUT);
    /* set gpio mode */
    mt_set_gpio_mode(GPIO_COMBO_PMU_EN_PIN, GPIO_MODE_GPIO);
    mt_set_gpio_mode(GPIO_COMBO_RST_PIN, GPIO_MODE_GPIO);

    /* SYSRST_B low */
    mt_set_gpio_out(GPIO_COMBO_RST_PIN, GPIO_OUT_ZERO);
    /* PMU_EN low */
    mt_set_gpio_out(GPIO_COMBO_PMU_EN_PIN, GPIO_OUT_ZERO);
    msleep(MT6620_OFF_TIME);

    /* PMU_EN high, SYSRST_B low */
    mt_set_gpio_out(GPIO_COMBO_PMU_EN_PIN, GPIO_OUT_ONE);
    msleep(MT6620_RST_TIME);

    /* SYSRST_B high */
    mt_set_gpio_out(GPIO_COMBO_RST_PIN, GPIO_OUT_ONE);
    msleep(MT6620_STABLE_TIME);

    /* BT PCM bus default mode. Real control is done by audio and mt_combo.c */
    mt_combo_audio_ctrl_ex(COMBO_AUDIO_STATE_1, 0);

    /* EINT1 for BGF_INT_B */
    mt_set_gpio_mode(GPIO_COMBO_BGF_EINT_PIN, GPIO_COMBO_BGF_EINT_PIN_M_GPIO);
    mt_set_gpio_pull_enable(GPIO_COMBO_BGF_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_COMBO_BGF_EINT_PIN, GPIO_PULL_UP);
    mt_set_gpio_mode(GPIO_COMBO_BGF_EINT_PIN, GPIO_COMBO_BGF_EINT_PIN_M_EINT);

    /* request IRQ (EINT1) */
    mt_combo_bgf_request_irq(NULL);
    mt_combo_binfo_ctrl(COMBO_BOARD_INFO_BTYPE, COMBO_BOARD_TYPE_ZTEMT73V2);
    
    /*Chaozhong.Liang: I2S1 PIN settings for FM Tx - set GPIO 200, 201, 202 to I2S0 function (GPIO_MODE_03)*/
#ifndef FM_ANALOG_INPUT
	#ifndef GPIO_I2S1_CK_PIN    
		mt_set_gpio_mode(GPIO200, GPIO_MODE_03);
		mt_set_gpio_mode(GPIO201, GPIO_MODE_03);
		mt_set_gpio_mode(GPIO202, GPIO_MODE_03);
	#else
		mt_set_gpio_mode(GPIO_I2S1_CK_PIN, GPIO_I2S1_CK_PIN_M_I2S0_CK);
		mt_set_gpio_mode(GPIO_I2S1_WS_PIN, GPIO_MODE_03);//why GPIO_I2S1_WS_PIN_M_I2S0_DAT is used , not GPIO_I2S1_WS_PIN_M_I2S0_WS?
		mt_set_gpio_mode(GPIO_I2S1_DAT_PIN, GPIO_MODE_03);//why GPIO_I2S1_DAT_PIN_M_I2S0_WS is used, not GPIO_I2S1_DAT_PIN_M_I2S0_DAT?
	#endif

#endif
    printk(KERN_INFO "[mt6620] power on \n");

    return;
}
EXPORT_SYMBOL(mt6620_power_on);

void mt6620_power_off(void)
{
    printk(KERN_INFO "[mt6620] power off\n");

    //printk(KERN_INFO "[mt6620] mt_combo_bgf_disable_irq\n");
    mt_combo_bgf_disable_irq();

    //printk(KERN_INFO "[mt6620] set BGF_EINT input pull down\n");
    mt_set_gpio_mode(GPIO_COMBO_BGF_EINT_PIN, GPIO_COMBO_BGF_EINT_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_COMBO_BGF_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_select(GPIO_COMBO_BGF_EINT_PIN, GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO_COMBO_BGF_EINT_PIN, GPIO_PULL_ENABLE);

#ifdef GPIO_COMBO_ALL_EINT_PIN
    //printk(KERN_INFO "[mt6620] set ALL_EINT input pull down\n");
    mt_set_gpio_mode(GPIO_COMBO_ALL_EINT_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_COMBO_ALL_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_select(GPIO_COMBO_ALL_EINT_PIN, GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO_COMBO_ALL_EINT_PIN, GPIO_PULL_ENABLE);
#endif

    //printk(KERN_INFO "[mt6620] set COMBO_AUDIO_STATE_0\n");
    mt_combo_audio_ctrl_ex(COMBO_AUDIO_STATE_0, 0);

    //printk(KERN_INFO "[mt6620] set I2S1(gpio 200,201,202) input pull down\n");
#ifndef FM_ANALOG_INPUT
	#ifndef GPIO_I2S1_CK_PIN     
		mt_set_gpio_mode(GPIO200, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO200, GPIO_DIR_IN);
		mt_set_gpio_pull_select(GPIO200, GPIO_PULL_DOWN);
		mt_set_gpio_pull_enable(GPIO200, GPIO_PULL_ENABLE);
		mt_set_gpio_mode(GPIO201, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO201, GPIO_DIR_IN);
		mt_set_gpio_pull_select(GPIO201, GPIO_PULL_DOWN);
		mt_set_gpio_pull_enable(GPIO201, GPIO_PULL_ENABLE);
		mt_set_gpio_mode(GPIO202, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO202, GPIO_DIR_IN);
		mt_set_gpio_pull_select(GPIO202, GPIO_PULL_DOWN);
		mt_set_gpio_pull_enable(GPIO202, GPIO_PULL_ENABLE);
	#else
		mt_set_gpio_mode(GPIO_I2S1_CK_PIN, GPIO_I2S1_CK_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_I2S1_CK_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_select(GPIO_I2S1_CK_PIN, GPIO_PULL_DOWN);
		mt_set_gpio_pull_enable(GPIO_I2S1_CK_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_mode(GPIO_I2S1_WS_PIN, GPIO_I2S1_WS_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_I2S1_WS_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_select(GPIO_I2S1_WS_PIN, GPIO_PULL_DOWN);
		mt_set_gpio_pull_enable(GPIO_I2S1_WS_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_mode(GPIO_I2S1_DAT_PIN, GPIO_I2S1_DAT_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_I2S1_DAT_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_select(GPIO_I2S1_DAT_PIN, GPIO_PULL_DOWN);
		mt_set_gpio_pull_enable(GPIO_I2S1_DAT_PIN, GPIO_PULL_ENABLE);		
	#endif
#endif
   // printk(KERN_INFO "[mt6620] set SYSRST_B 0 and PMU_EN 0 \n");
    /* SYSRST_B low */
    mt_set_gpio_out(GPIO_COMBO_RST_PIN, GPIO_OUT_ZERO);
    /* PMU_EN low */
    mt_set_gpio_out(GPIO_COMBO_PMU_EN_PIN, GPIO_OUT_ZERO);

#if defined(CONFIG_MTK_COMBO_SDIO_SLOT) && (CONFIG_MTK_COMBO_SDIO_SLOT == 1)
    //printk(KERN_INFO "[mt6620] pull down sd1 bus(gpio62~68)\n");
    mt_set_gpio_pull_select(GPIO62, GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO62, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO63, GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO63, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO64, GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO64, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO65, GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO65, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO66, GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO66, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO67, GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO67, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO68, GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO68, GPIO_PULL_ENABLE);
#endif

    //printk(KERN_INFO "[mt6620] set UART GPIO Mode output 0\n");
    mt_set_gpio_mode(GPIO_UART_URXD3_PIN, GPIO_UART_URXD3_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_UART_URXD3_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_UART_URXD3_PIN, GPIO_OUT_ZERO);

    mt_set_gpio_mode(GPIO_UART_UTXD3_PIN, GPIO_UART_UTXD3_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_UART_UTXD3_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_UART_UTXD3_PIN, GPIO_OUT_ZERO);

    //printk(KERN_INFO "[mt6620] disable RTC GPIO \n");
    printk("[mt6620]not to rtc_gpio_disable_32k(RTC_GPIO_USER_GPS)  \n");
    //rtc_gpio_export_32k(false);

    //printk(KERN_INFO "[mt6620] disable external LDO\n");
    /* external LDO_EN high */
    mt_set_gpio_out(GPIO_COMBO_6620_LDO_EN_PIN, GPIO_OUT_ZERO);
    
    return;
}
EXPORT_SYMBOL(mt6620_power_off);

static void combo_bt_pcm_pin_on(void)
{
    mt_set_gpio_mode(GPIO_PCM_DAICLK_PIN, GPIO_PCM_DAICLK_PIN_M_CLK);
    mt_set_gpio_mode(GPIO_PCM_DAIPCMOUT_PIN, GPIO_PCM_DAIPCMOUT_PIN_M_DAIPCMOUT);
    mt_set_gpio_mode(GPIO_PCM_DAIPCMIN_PIN, GPIO_PCM_DAIPCMIN_PIN_M_DAIPCMIN);
    mt_set_gpio_mode(GPIO_PCM_DAISYNC_PIN, GPIO_PCM_DAISYNC_PIN_M_DAISYNC);	
}

static void combo_bt_pcm_pin_off(void)
{
    mt_set_gpio_mode(GPIO_PCM_DAICLK_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_PCM_DAICLK_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_PCM_DAICLK_PIN, GPIO_OUT_ZERO);

    mt_set_gpio_mode(GPIO_PCM_DAIPCMOUT_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_PCM_DAIPCMOUT_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_PCM_DAIPCMOUT_PIN, GPIO_OUT_ZERO);

    mt_set_gpio_mode(GPIO_PCM_DAIPCMIN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_PCM_DAIPCMIN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_PCM_DAIPCMIN_PIN, GPIO_OUT_ZERO);

    mt_set_gpio_mode(GPIO_PCM_DAISYNC_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_PCM_DAISYNC_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_PCM_DAISYNC_PIN, GPIO_OUT_ZERO);
}

static void combo_fm_i2s_pin_on(void)
{
#ifndef FM_ANALOG_INPUT
    mt_set_gpio_mode(GPIO_I2S1_CK_PIN, GPIO_MODE_03);	//GPIO_MODE_03->I2S0 mode
    mt_set_gpio_mode(GPIO_I2S1_WS_PIN, GPIO_MODE_03);	//GPIO_MODE_03->I2S0 mode
    mt_set_gpio_mode(GPIO_I2S1_DAT_PIN, GPIO_MODE_03);	//GPIO_MODE_03->I2S0 mode
#else
	printk(KERN_INFO "[MT6620]warnning:FM analog mode is set, no I2S GPIO settings should be modified by combo driver\n");	
#endif
}

static void combo_fm_i2s_pin_off(void)
{
#ifndef FM_ANALOG_INPUT
    mt_set_gpio_mode(GPIO_I2S1_CK_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_I2S1_CK_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_I2S1_CK_PIN, GPIO_OUT_ZERO);

    mt_set_gpio_mode(GPIO_I2S1_WS_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_I2S1_WS_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_I2S1_WS_PIN, GPIO_OUT_ZERO);

    mt_set_gpio_mode(GPIO_I2S1_DAT_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_I2S1_DAT_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_I2S1_DAT_PIN, GPIO_OUT_ZERO);
#else
	printk(KERN_INFO "[MT6620]warnning:FM analog mode is set, no I2S GPIO settings should be modified by combo driver\n");	
#endif
}



void combo_audio_pin_conf(COMBO_AUDIO_STATE state)
{
    printk(KERN_INFO "combo_audio_pin_conf, state = [%d]\n", state);
    switch(state)
	{
        case COMBO_AUDIO_STATE_0:
        /*BT_PCM_OFF*/ 
            combo_bt_pcm_pin_off();
        /*FM_I2S1_OFF*/
            combo_fm_i2s_pin_off();
        break;
        case COMBO_AUDIO_STATE_1:
        /* BT_PCM_ON */
            combo_bt_pcm_pin_on();
        /*FM_I2S1_OFF*/
            combo_fm_i2s_pin_off();
        break;
        case COMBO_AUDIO_STATE_2:
        /*FM_I2S_ON*/
            combo_fm_i2s_pin_on();
        /*BT_PCM_OFF*/ 
            combo_bt_pcm_pin_off();
        break;
        case COMBO_AUDIO_STATE_3:
        /*FM_I2S_ON*/
            combo_fm_i2s_pin_on();
        /* BT_PCM_ON */
            combo_bt_pcm_pin_on();
        break;
        default:
        break;
    }	
}

EXPORT_SYMBOL(combo_audio_pin_conf);

    #if defined(CONFIG_MTK_COMBO_SDIO_SLOT)
static void combo_sdio_enable_eirq(void)
{
    mt65xx_eint_unmask(combo_sdio_eint_num);/* CUST_EINT_WIFI_NUM */
}

static void combo_sdio_disable_eirq(void)
{
    mt65xx_eint_mask(combo_sdio_eint_num); /* CUST_EINT_WIFI_NUM */
}

static void combo_sdio_eirq_handler_stub(void)
{
    if (combo_sdio_eirq_handler) {
        combo_sdio_eirq_handler(combo_sdio_eirq_data);
    }
}

static void combo_sdio_request_eirq(sdio_irq_handler_t irq_handler, void *data)
{
    mt65xx_eint_set_sens(combo_sdio_eint_num, CUST_EINT_WIFI_SENSITIVE); /*CUST_EINT_WIFI_NUM */
    mt65xx_eint_set_hw_debounce(combo_sdio_eint_num, CUST_EINT_WIFI_DEBOUNCE_CN); /*CUST_EINT_WIFI_NUM */
    mt65xx_eint_registration(combo_sdio_eint_num/*CUST_EINT_WIFI_NUM */,
        CUST_EINT_WIFI_DEBOUNCE_EN,
        CUST_EINT_WIFI_POLARITY,
        combo_sdio_eirq_handler_stub,
        0);
    mt65xx_eint_mask(combo_sdio_eint_num);/*CUST_EINT_WIFI_NUM */

    combo_sdio_eirq_handler = irq_handler;
    combo_sdio_eirq_data    = data;
}

static void combo_sdio_register_pm(pm_callback_t pm_cb, void *data)
{
    /*printk( KERN_INFO "combo_sdio_register_pm (0x%p, 0x%p)\n", pm_cb, data);*/
    /* register pm change callback */
    combo_sdio_pm_cb = pm_cb;
    combo_sdio_pm_data = data;
}

static void combo_sdio_on (int sdio_port_num) {
    pm_message_t state = { .event = PM_EVENT_USER_RESUME };

    printk(KERN_INFO "combo_sdio_on (%d) \n", sdio_port_num);

    /* 1. disable sdio eirq */
    combo_sdio_disable_eirq();
    mt_set_gpio_pull_enable(combo_sdio_eint_pin, GPIO_PULL_DISABLE); /* GPIO_WIFI_EINT_PIN */
    mt_set_gpio_mode(combo_sdio_eint_pin, combo_sdio_eint_m_eint); /* EINT mode */

    /* 2. call sd callback */
    if (combo_sdio_pm_cb) {
        //printk(KERN_INFO "combo_sdio_pm_cb(PM_EVENT_USER_RESUME, 0x%p, 0x%p) \n", combo_sdio_pm_cb, combo_sdio_pm_data);
        combo_sdio_pm_cb(state, combo_sdio_pm_data);
    }
    else {
        printk(KERN_WARNING "combo_sdio_on no sd callback!!\n");
    }
}

static void combo_sdio_off (int sdio_port_num) {
    pm_message_t state = { .event = PM_EVENT_USER_SUSPEND };

    printk(KERN_INFO "combo_sdio_off (%d) \n", sdio_port_num);

    /* 1. call sd callback */
    if (combo_sdio_pm_cb) {
        //printk(KERN_INFO "combo_sdio_off(PM_EVENT_USER_SUSPEND, 0x%p, 0x%p) \n", combo_sdio_pm_cb, combo_sdio_pm_data);
        combo_sdio_pm_cb(state, combo_sdio_pm_data);
    }
    else {
        printk(KERN_WARNING "combo_sdio_off no sd callback!!\n");
    }

    /* 2. disable sdio eirq */
    combo_sdio_disable_eirq();
    /*printk(KERN_INFO "[mt6620] set WIFI_EINT input pull down\n");*/
    mt_set_gpio_mode(combo_sdio_eint_pin, combo_sdio_eint_m_gpio); /* GPIO mode */
    mt_set_gpio_dir(combo_sdio_eint_pin, GPIO_DIR_IN);
    mt_set_gpio_pull_select(combo_sdio_eint_pin, GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(combo_sdio_eint_pin, GPIO_PULL_ENABLE);
}

int mt_combo_sdio_ctrl (unsigned int sdio_port_num, unsigned int on) {
    if ((sdio_port_num >= 3) || (combo_port_pwr_map[sdio_port_num] == 0xFF) ) {
        /* invalid sdio port number or slot mapping */
        printk(KERN_WARNING "mt_combo_sdio_ctrl invalid port(%d, %d)\n", sdio_port_num, combo_port_pwr_map[sdio_port_num]);
        return -1;
    }
    /*printk(KERN_INFO "mt_combo_sdio_ctrl (%d, %d)\n", sdio_port_num, on);*/

    if (!combo_port_pwr_map[sdio_port_num] && on) {
        /* off -> on */
        combo_sdio_on(sdio_port_num);
        combo_port_pwr_map[sdio_port_num] = 1;
    }
    else if (combo_port_pwr_map[sdio_port_num] && !on) {
        /* on -> off */
        combo_sdio_off(sdio_port_num);
        combo_port_pwr_map[sdio_port_num] = 0;
    }
    else {
        return -2;
    }
    return 0;
}

    #else /* !defined(CONFIG_MTK_COMBO_SDIO_SLOT) */

int mt_combo_sdio_ctrl (unsigned int sdio_port_num, unsigned int on) {
    printk(KERN_WARNING "mt_combo_sdio_ctrl but CONFIG_MTK_COMBO_SDIO_SLOT undefined!\n");
    return -1;
}

    #endif /* end of defined(CONFIG_MTK_COMBO_SDIO_SLOT) */
EXPORT_SYMBOL(mt_combo_sdio_ctrl);

#endif /* end of  defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE) */

#if defined(CONFIG_WLAN)
    #if !defined(CONFIG_MTK_COMBO_SDIO_SLOT)
static void mt_wifi_enable_irq(void)
{
    mt65xx_eint_unmask(CUST_EINT_WIFI_NUM);
}

static void mt_wifi_disable_irq(void)
{
    mt65xx_eint_mask(CUST_EINT_WIFI_NUM);
}

static void mt_wifi_eirq_handler(void)
{
    if (mt_wifi_irq_handler) {
        mt_wifi_irq_handler(mt_wifi_irq_data);
    }
}

static void mt_wifi_request_irq(sdio_irq_handler_t irq_handler, void *data)
{
    mt65xx_eint_set_sens(CUST_EINT_WIFI_NUM, CUST_EINT_WIFI_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_WIFI_NUM, CUST_EINT_WIFI_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_WIFI_NUM,
        CUST_EINT_WIFI_DEBOUNCE_EN,
        CUST_EINT_WIFI_POLARITY,
        mt_wifi_eirq_handler,
        0);
    mt65xx_eint_mask(CUST_EINT_WIFI_NUM);

    mt_wifi_irq_handler = irq_handler;
    mt_wifi_irq_data    = data;
}

static void mt_wifi_register_pm(pm_callback_t pm_cb, void *data)
{
    /* register pm change callback */
    mt_wifi_pm_cb = pm_cb;
    mt_wifi_pm_data = data;
}

    #endif /* end of !defined(CONFIG_MTK_COMBO_SDIO_SLOT) */

int mt_wifi_resume(pm_message_t state)
{
    int evt = state.event;

    if (evt != PM_EVENT_USER_RESUME && evt != PM_EVENT_RESUME) {
        return -1;
    }

    /*printk(KERN_INFO "[WIFI] %s Resume\n", evt == PM_EVENT_RESUME ? "PM":"USR");*/

#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE)
    /* combo chip product: notify combo driver to turn on Wi-Fi */
    mt_combo_func_ctrl(COMBO_FUNC_TYPE_WIFI, 1);

#endif

    return 0;
}

int mt_wifi_suspend(pm_message_t state)
{
    int evt = state.event;
#if defined(CONFIG_MTK_COMBO_SDIO_SLOT)
    static int is_1st_suspend_from_boot = 1;
#endif

    if (evt != PM_EVENT_USER_SUSPEND && evt != PM_EVENT_SUSPEND) {
        return -1;
    }

#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE)
    #if defined(CONFIG_MTK_COMBO_SDIO_SLOT)
    /* combo chip product: notify combo driver to turn on Wi-Fi */
    if (is_1st_suspend_from_boot) {
        pm_message_t state = { .event = PM_EVENT_USER_SUSPEND };

        if (combo_sdio_pm_cb) {
            is_1st_suspend_from_boot = 0;
            /*              *** IMPORTANT DEPENDENDY***
            RFKILL: set wifi and bt suspend by default in probe()
            MT6573-SD: sd host is added to MMC stack and suspend is ZERO by default
            (which means NOT suspended).

            When boot up, RFKILL will set wifi off and this function gets
            called. In order to successfully resume wifi at 1st time, pm_cb here
            shall be called once to let MT6573-SD do sd host suspend and remove
            sd host from MMC. Then wifi can be turned on successfully.

            Boot->SD host added to MMC (suspend=0)->RFKILL set wifi off
            ->SD host removed from MMC (suspend=1)->RFKILL set wifi on
            */
            printk(KERN_INFO "1st mt_wifi_suspend (PM_EVENT_USER_SUSPEND) \n");
            combo_sdio_pm_cb(state, combo_sdio_pm_data);
        }
        else {
            printk(KERN_WARNING "1st mt_wifi_suspend but no sd callback!!\n");
        }
    }
    else {
        /* combo chip product, notify combo driver */
        mt_combo_func_ctrl(COMBO_FUNC_TYPE_WIFI, 0);
    }
    #endif
#endif
    return 0;
}

void mt_wifi_power_on(void)
{
    pm_message_t state = { .event = PM_EVENT_USER_RESUME };

    (void)mt_wifi_resume(state);
}
EXPORT_SYMBOL(mt_wifi_power_on);

void mt_wifi_power_off(void)
{
    pm_message_t state = { .event = PM_EVENT_USER_SUSPEND };

    (void)mt_wifi_suspend(state);
}
EXPORT_SYMBOL(mt_wifi_power_off);

#endif /* end of defined(CONFIG_WLAN) */


/*=======================================================================*/
/* Board Devices Capability                                              */
/*=======================================================================*/
#if defined(CFG_DEV_MSDC0)
struct mt6573_sd_host_hw mt6573_sd0_hw = {
    .clk_src        = MSDC_CLKSRC_98MHZ,
    .cmd_edge       = EDGE_RISING,
    .data_edge      = EDGE_RISING,
    .cmd_odc        = MSDC_ODC_8MA,
    .data_odc       = MSDC_ODC_8MA,
    .cmd_slew_rate  = MSDC_ODC_SLEW_SLOW,
    .data_slew_rate = MSDC_ODC_SLEW_SLOW,
    .cmd_pull_res   = MSDC_PULL_RES_23K,
    .dat_pull_res   = MSDC_PULL_RES_23K,
    .clk_pull_res   = MSDC_PULL_RES_23K,
    .rst_wp_pull_res= MSDC_PULL_RES_23K,
    .data_pins      = 4,
    .data_offset    = 0,
    .flags          = MSDC_SYS_SUSPEND | MSDC_CD_PIN_EN | MSDC_REMOVABLE | MSDC_HIGHSPEED,
};
#endif
#if defined(CFG_DEV_MSDC1)
    #if defined(CONFIG_MTK_COMBO_SDIO_SLOT) && (CONFIG_MTK_COMBO_SDIO_SLOT == 1)
struct mt6573_sd_host_hw mt6573_sd1_hw = {
    .clk_src        = MSDC_CLKSRC_81MHZ, /* Use 81/2 MHz */
    .cmd_edge       = EDGE_RISING,
    .data_edge      = EDGE_RISING,
    .cmd_odc        = MSDC_ODC_8MA,
    .data_odc       = MSDC_ODC_8MA,
    .cmd_slew_rate  = MSDC_ODC_SLEW_FAST,
    .data_slew_rate = MSDC_ODC_SLEW_FAST,
    .cmd_pull_res   = MSDC_PULL_RES_47K,
    .dat_pull_res   = MSDC_PULL_RES_47K,
    .clk_pull_res   = MSDC_PULL_RES_47K,
    .rst_wp_pull_res= MSDC_PULL_RES_47K,
    .data_pins      = 4,
    .data_offset    = 0,
    .flags          = MSDC_EXT_SDIO_IRQ | MSDC_HIGHSPEED,
    .request_sdio_eirq = combo_sdio_request_eirq,
    .enable_sdio_eirq  = combo_sdio_enable_eirq,
    .disable_sdio_eirq = combo_sdio_disable_eirq,
    .register_pm       = combo_sdio_register_pm,
};
    #else
struct mt6573_sd_host_hw mt6573_sd1_hw = {
    .clk_src        = MSDC_CLKSRC_81MHZ,
    .cmd_edge       = EDGE_RISING,
    .data_edge      = EDGE_RISING,
    .cmd_odc        = MSDC_ODC_8MA,
    .data_odc       = MSDC_ODC_8MA,
    .cmd_slew_rate  = MSDC_ODC_SLEW_FAST,
    .data_slew_rate = MSDC_ODC_SLEW_FAST,
    .cmd_pull_res   = MSDC_PULL_RES_47K,
    .dat_pull_res   = MSDC_PULL_RES_47K,
    .clk_pull_res   = MSDC_PULL_RES_47K,
    .rst_wp_pull_res= MSDC_PULL_RES_47K,
    .data_pins      = 4,
    .data_offset    = 0,
    .flags          = MSDC_EXT_SDIO_IRQ | MSDC_HIGHSPEED,
    .request_sdio_eirq = mt_wifi_request_irq,
    .enable_sdio_eirq  = mt_wifi_enable_irq,
    .disable_sdio_eirq = mt_wifi_disable_irq,
    .register_pm       = mt_wifi_register_pm,
};
    #endif
#endif
#if defined(CFG_DEV_MSDC2)
struct mt6573_sd_host_hw mt6573_sd2_hw = {
    .clk_src        = MSDC_CLKSRC_98MHZ,
    .cmd_edge       = EDGE_RISING,
    .data_edge      = EDGE_RISING,
    .cmd_odc        = MSDC_ODC_16MA,
    .data_odc       = MSDC_ODC_12MA,
    .cmd_slew_rate  = MSDC_ODC_SLEW_SLOW,
    .data_slew_rate = MSDC_ODC_SLEW_SLOW,
    .cmd_pull_res   = MSDC_PULL_RES_23K,
    .dat_pull_res   = MSDC_PULL_RES_23K,
    .clk_pull_res   = MSDC_PULL_RES_23K,
    .rst_wp_pull_res= MSDC_PULL_RES_23K,
    .data_pins      = 4,
    .data_offset    = 0,
    .flags          = MSDC_SYS_SUSPEND,
};
#endif
#if defined(CFG_DEV_MSDC3)
struct mt6573_sd_host_hw mt6573_sd3_hw = {
    .clk_src        = MSDC_CLKSRC_98MHZ,
    .cmd_edge       = EDGE_RISING,
    .data_edge      = EDGE_RISING,
    .cmd_odc        = MSDC_ODC_16MA,
    .data_odc       = MSDC_ODC_12MA,
    .cmd_slew_rate  = MSDC_ODC_SLEW_SLOW,
    .data_slew_rate = MSDC_ODC_SLEW_SLOW,
    .cmd_pull_res   = MSDC_PULL_RES_23K,
    .dat_pull_res   = MSDC_PULL_RES_23K,
    .clk_pull_res   = MSDC_PULL_RES_23K,
    .rst_wp_pull_res= MSDC_PULL_RES_23K,
    .data_pins      = 4,
    .data_offset    = 0,
    .flags          = MSDC_SYS_SUSPEND,
};
#endif

/* MT6573 NAND Driver */
#if defined(CONFIG_MTK_MTD_NAND)
struct mt6573_nand_host_hw mt6573_nand_hw = {
    .nfi_bus_width          = 8,
	.nfi_access_timing		= NFI_DEFAULT_ACCESS_TIMING,
	.nfi_cs_num				= NFI_CS_NUM,
	.nand_sec_size			= 512,
	.nand_sec_shift			= 9,
	.nand_ecc_size			= 2048,
	.nand_ecc_bytes			= 32,
	.nand_ecc_mode			= NAND_ECC_HW,
};
#endif
