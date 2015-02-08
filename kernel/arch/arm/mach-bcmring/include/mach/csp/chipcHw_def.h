

#ifndef CHIPC_DEF_H
#define CHIPC_DEF_H

/* ---- Include Files ----------------------------------------------------- */

#include <csp/stdint.h>
#include <csp/errno.h>
#include <csp/reg.h>
#include <mach/csp/chipcHw_reg.h>

/* ---- Public Constants and Types ---------------------------------------- */

/* Set 1 to configure DDR/VPM phase alignment by HW */
#define chipcHw_DDR_HW_PHASE_ALIGN    0
#define chipcHw_VPM_HW_PHASE_ALIGN    0

typedef uint32_t chipcHw_freq;

/* Configurable miscellaneous clocks */
typedef enum {
	chipcHw_CLOCK_DDR,	/* DDR PHY Clock */
	chipcHw_CLOCK_ARM,	/* ARM Clock */
	chipcHw_CLOCK_ESW,	/* Ethernet Switch Clock */
	chipcHw_CLOCK_VPM,	/* VPM Clock */
	chipcHw_CLOCK_ESW125,	/* Ethernet MII Clock */
	chipcHw_CLOCK_UART,	/* UART Clock */
	chipcHw_CLOCK_SDIO0,	/* SDIO 0 Clock */
	chipcHw_CLOCK_SDIO1,	/* SDIO 1 Clock */
	chipcHw_CLOCK_SPI,	/* SPI Clock */
	chipcHw_CLOCK_ETM,	/* ARM ETM Clock */

	chipcHw_CLOCK_BUS,	/* BUS Clock */
	chipcHw_CLOCK_OTP,	/* OTP Clock */
	chipcHw_CLOCK_I2C,	/* I2C Host Clock */
	chipcHw_CLOCK_I2S0,	/* I2S 0 Host Clock */
	chipcHw_CLOCK_RTBUS,	/* DDR PHY Configuration Clock */
	chipcHw_CLOCK_APM100,	/* APM100 Clock */
	chipcHw_CLOCK_TSC,	/* Touch screen Clock */
	chipcHw_CLOCK_LED,	/* LED Clock */

	chipcHw_CLOCK_USB,	/* USB Clock */
	chipcHw_CLOCK_LCD,	/* LCD CLock */
	chipcHw_CLOCK_APM,	/* APM Clock */

	chipcHw_CLOCK_I2S1,	/* I2S 1 Host Clock */
} chipcHw_CLOCK_e;

/* System booting strap options */
typedef enum {
	chipcHw_BOOT_DEVICE_UART = chipcHw_STRAPS_BOOT_DEVICE_UART,
	chipcHw_BOOT_DEVICE_SERIAL_FLASH =
	    chipcHw_STRAPS_BOOT_DEVICE_SERIAL_FLASH,
	chipcHw_BOOT_DEVICE_NOR_FLASH_16 =
	    chipcHw_STRAPS_BOOT_DEVICE_NOR_FLASH_16,
	chipcHw_BOOT_DEVICE_NAND_FLASH_8 =
	    chipcHw_STRAPS_BOOT_DEVICE_NAND_FLASH_8,
	chipcHw_BOOT_DEVICE_NAND_FLASH_16 =
	    chipcHw_STRAPS_BOOT_DEVICE_NAND_FLASH_16
} chipcHw_BOOT_DEVICE_e;

/* System booting modes */
typedef enum {
	chipcHw_BOOT_MODE_NORMAL = chipcHw_STRAPS_BOOT_MODE_NORMAL,
	chipcHw_BOOT_MODE_DBG_SW = chipcHw_STRAPS_BOOT_MODE_DBG_SW,
	chipcHw_BOOT_MODE_DBG_BOOT = chipcHw_STRAPS_BOOT_MODE_DBG_BOOT,
	chipcHw_BOOT_MODE_NORMAL_QUIET = chipcHw_STRAPS_BOOT_MODE_NORMAL_QUIET
} chipcHw_BOOT_MODE_e;

/* NAND Flash page size strap options */
typedef enum {
	chipcHw_NAND_PAGESIZE_512 = chipcHw_STRAPS_NAND_PAGESIZE_512,
	chipcHw_NAND_PAGESIZE_2048 = chipcHw_STRAPS_NAND_PAGESIZE_2048,
	chipcHw_NAND_PAGESIZE_4096 = chipcHw_STRAPS_NAND_PAGESIZE_4096,
	chipcHw_NAND_PAGESIZE_EXT = chipcHw_STRAPS_NAND_PAGESIZE_EXT
} chipcHw_NAND_PAGESIZE_e;

/* GPIO Pin function */
typedef enum {
	chipcHw_GPIO_FUNCTION_KEYPAD = chipcHw_REG_GPIO_MUX_KEYPAD,
	chipcHw_GPIO_FUNCTION_I2CH = chipcHw_REG_GPIO_MUX_I2CH,
	chipcHw_GPIO_FUNCTION_SPI = chipcHw_REG_GPIO_MUX_SPI,
	chipcHw_GPIO_FUNCTION_UART = chipcHw_REG_GPIO_MUX_UART,
	chipcHw_GPIO_FUNCTION_LEDMTXP = chipcHw_REG_GPIO_MUX_LEDMTXP,
	chipcHw_GPIO_FUNCTION_LEDMTXS = chipcHw_REG_GPIO_MUX_LEDMTXS,
	chipcHw_GPIO_FUNCTION_SDIO0 = chipcHw_REG_GPIO_MUX_SDIO0,
	chipcHw_GPIO_FUNCTION_SDIO1 = chipcHw_REG_GPIO_MUX_SDIO1,
	chipcHw_GPIO_FUNCTION_PCM = chipcHw_REG_GPIO_MUX_PCM,
	chipcHw_GPIO_FUNCTION_I2S = chipcHw_REG_GPIO_MUX_I2S,
	chipcHw_GPIO_FUNCTION_ETM = chipcHw_REG_GPIO_MUX_ETM,
	chipcHw_GPIO_FUNCTION_DEBUG = chipcHw_REG_GPIO_MUX_DEBUG,
	chipcHw_GPIO_FUNCTION_MISC = chipcHw_REG_GPIO_MUX_MISC,
	chipcHw_GPIO_FUNCTION_GPIO = chipcHw_REG_GPIO_MUX_GPIO
} chipcHw_GPIO_FUNCTION_e;

/* PIN Output slew rate */
typedef enum {
	chipcHw_PIN_SLEW_RATE_HIGH = chipcHw_REG_SLEW_RATE_HIGH,
	chipcHw_PIN_SLEW_RATE_NORMAL = chipcHw_REG_SLEW_RATE_NORMAL
} chipcHw_PIN_SLEW_RATE_e;

/* PIN Current drive strength */
typedef enum {
	chipcHw_PIN_CURRENT_STRENGTH_2mA = chipcHw_REG_CURRENT_STRENGTH_2mA,
	chipcHw_PIN_CURRENT_STRENGTH_4mA = chipcHw_REG_CURRENT_STRENGTH_4mA,
	chipcHw_PIN_CURRENT_STRENGTH_6mA = chipcHw_REG_CURRENT_STRENGTH_6mA,
	chipcHw_PIN_CURRENT_STRENGTH_8mA = chipcHw_REG_CURRENT_STRENGTH_8mA,
	chipcHw_PIN_CURRENT_STRENGTH_10mA = chipcHw_REG_CURRENT_STRENGTH_10mA,
	chipcHw_PIN_CURRENT_STRENGTH_12mA = chipcHw_REG_CURRENT_STRENGTH_12mA
} chipcHw_PIN_CURRENT_STRENGTH_e;

/* PIN Pull up register settings */
typedef enum {
	chipcHw_PIN_PULL_NONE = chipcHw_REG_PULL_NONE,
	chipcHw_PIN_PULL_UP = chipcHw_REG_PULL_UP,
	chipcHw_PIN_PULL_DOWN = chipcHw_REG_PULL_DOWN
} chipcHw_PIN_PULL_e;

/* PIN input type settings */
typedef enum {
	chipcHw_PIN_INPUTTYPE_CMOS = chipcHw_REG_INPUTTYPE_CMOS,
	chipcHw_PIN_INPUTTYPE_ST = chipcHw_REG_INPUTTYPE_ST
} chipcHw_PIN_INPUTTYPE_e;

/* Allow/Disalow the support of spread spectrum  */
typedef enum {
	chipcHw_SPREAD_SPECTRUM_DISALLOW,	/* Spread spectrum support is not allowed */
	chipcHw_SPREAD_SPECTRUM_ALLOW	/* Spread spectrum support is allowed */
} chipcHw_SPREAD_SPECTRUM_e;

typedef struct {
	chipcHw_SPREAD_SPECTRUM_e ssSupport;	/* Allow/Disalow to support spread spectrum.
						   If supported, call chipcHw_enableSpreadSpectrum ()
						   to activate the spread spectrum with desired spread. */
	uint32_t pllVcoFreqHz;	/* PLL VCO frequency in Hz */
	uint32_t pll2VcoFreqHz;	/* PLL2 VCO frequency in Hz */
	uint32_t busClockFreqHz;	/* Bus clock frequency in Hz */
	uint32_t armBusRatio;	/* ARM clock : Bus clock */
	uint32_t vpmBusRatio;	/* VPM clock : Bus clock */
	uint32_t ddrBusRatio;	/* DDR clock : Bus clock */
} chipcHw_INIT_PARAM_t;

/* CHIP revision number */
typedef enum {
	chipcHw_REV_NUMBER_A0 = chipcHw_REG_REV_A0,
	chipcHw_REV_NUMBER_B0 = chipcHw_REG_REV_B0
} chipcHw_REV_NUMBER_e;

typedef enum {
	chipcHw_VPM_HW_PHASE_INTR_DISABLE = chipcHw_REG_VPM_INTR_DISABLE,
	chipcHw_VPM_HW_PHASE_INTR_FAST = chipcHw_REG_VPM_INTR_FAST,
	chipcHw_VPM_HW_PHASE_INTR_MEDIUM = chipcHw_REG_VPM_INTR_MEDIUM,
	chipcHw_VPM_HW_PHASE_INTR_SLOW = chipcHw_REG_VPM_INTR_SLOW
} chipcHw_VPM_HW_PHASE_INTR_e;

typedef enum {
	chipcHw_DDR_HW_PHASE_MARGIN_STRICT,	/*  Strict margin for DDR phase align condition */
	chipcHw_DDR_HW_PHASE_MARGIN_MEDIUM,	/*  Medium margin for DDR phase align condition */
	chipcHw_DDR_HW_PHASE_MARGIN_WIDE	/*  Wider margin for DDR phase align condition */
} chipcHw_DDR_HW_PHASE_MARGIN_e;

typedef enum {
	chipcHw_VPM_HW_PHASE_MARGIN_STRICT,	/*  Strict margin for VPM phase align condition */
	chipcHw_VPM_HW_PHASE_MARGIN_MEDIUM,	/*  Medium margin for VPM phase align condition */
	chipcHw_VPM_HW_PHASE_MARGIN_WIDE	/*  Wider margin for VPM phase align condition */
} chipcHw_VPM_HW_PHASE_MARGIN_e;

#define chipcHw_XTAL_FREQ_Hz                    25000000	/* Reference clock frequency in Hz */

/* Programable pin defines */
#define chipcHw_PIN_GPIO(n)                     ((((n) >= 0) && ((n) < (chipcHw_GPIO_COUNT))) ? (n) : 0xFFFFFFFF)
									     /* GPIO pin 0 - 60 */
#define chipcHw_PIN_UARTTXD                     (chipcHw_GPIO_COUNT + 0)	/* UART Transmit */
#define chipcHw_PIN_NVI_A                       (chipcHw_GPIO_COUNT + 1)	/* NVI Interface */
#define chipcHw_PIN_NVI_D                       (chipcHw_GPIO_COUNT + 2)	/* NVI Interface */
#define chipcHw_PIN_NVI_OEB                     (chipcHw_GPIO_COUNT + 3)	/* NVI Interface */
#define chipcHw_PIN_NVI_WEB                     (chipcHw_GPIO_COUNT + 4)	/* NVI Interface */
#define chipcHw_PIN_NVI_CS                      (chipcHw_GPIO_COUNT + 5)	/* NVI Interface */
#define chipcHw_PIN_NVI_NAND_CSB                (chipcHw_GPIO_COUNT + 6)	/* NVI Interface */
#define chipcHw_PIN_NVI_FLASHWP                 (chipcHw_GPIO_COUNT + 7)	/* NVI Interface */
#define chipcHw_PIN_NVI_NAND_RDYB               (chipcHw_GPIO_COUNT + 8)	/* NVI Interface */
#define chipcHw_PIN_CL_DATA_0_17                (chipcHw_GPIO_COUNT + 9)	/* LCD Data 0 - 17 */
#define chipcHw_PIN_CL_DATA_18_20               (chipcHw_GPIO_COUNT + 10)	/* LCD Data 18 - 20 */
#define chipcHw_PIN_CL_DATA_21_23               (chipcHw_GPIO_COUNT + 11)	/* LCD Data 21 - 23 */
#define chipcHw_PIN_CL_POWER                    (chipcHw_GPIO_COUNT + 12)	/* LCD Power */
#define chipcHw_PIN_CL_ACK                      (chipcHw_GPIO_COUNT + 13)	/* LCD Ack */
#define chipcHw_PIN_CL_FP                       (chipcHw_GPIO_COUNT + 14)	/* LCD FP */
#define chipcHw_PIN_CL_LP                       (chipcHw_GPIO_COUNT + 15)	/* LCD LP */
#define chipcHw_PIN_UARTRXD                     (chipcHw_GPIO_COUNT + 16)	/* UART Receive */

/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */

/****************************************************************************/
/****************************************************************************/
void chipcHw_Init(chipcHw_INIT_PARAM_t *initParam	/*  [ IN ] Misc chip initialization parameter */
    ) __attribute__ ((section(".aramtext")));

/****************************************************************************/
/****************************************************************************/
void chipcHw_pll1Enable(uint32_t vcoFreqHz,	/*  [ IN ] VCO frequency in Hz */
			chipcHw_SPREAD_SPECTRUM_e ssSupport	/*  [ IN ] SS status */
    ) __attribute__ ((section(".aramtext")));

/****************************************************************************/
/****************************************************************************/
void chipcHw_pll2Enable(uint32_t vcoFreqHz	/*  [ IN ] VCO frequency in Hz */
    ) __attribute__ ((section(".aramtext")));

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_pll1Disable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_pll2Disable(void);

/****************************************************************************/
/****************************************************************************/
chipcHw_freq chipcHw_getClockFrequency(chipcHw_CLOCK_e clock	/*  [ IN ] Configurable clock */
    ) __attribute__ ((section(".aramtext")));

/****************************************************************************/
/****************************************************************************/
chipcHw_freq chipcHw_setClockFrequency(chipcHw_CLOCK_e clock,	/*  [ IN ] Configurable clock */
				       uint32_t freq	/*  [ IN ] Clock frequency in Hz */
    ) __attribute__ ((section(".aramtext")));

/****************************************************************************/
/****************************************************************************/
int chipcHw_vpmPhaseAlign(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_setClockEnable(chipcHw_CLOCK_e clock	/*  [ IN ] Configurable clock */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_setClockDisable(chipcHw_CLOCK_e clock	/*  [ IN ] Configurable clock */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_bypassClockEnable(chipcHw_CLOCK_e clock	/*  [ IN ] Configurable clock */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_bypassClockDisable(chipcHw_CLOCK_e clock	/*  [ IN ] Configurable clock */
    );

/****************************************************************************/
/****************************************************************************/
static inline uint32_t chipcHw_getChipId(void);

/****************************************************************************/
/****************************************************************************/
static inline uint32_t chipcHw_getChipProductId(void);

/****************************************************************************/
/****************************************************************************/
static inline chipcHw_REV_NUMBER_e chipcHw_getChipRevisionNumber(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_busInterfaceClockEnable(uint32_t mask	/*  [ IN ] Bit map of type  chipcHw_REG_BUS_CLOCK_XXXXX */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_busInterfaceClockDisable(uint32_t mask	/*  [ IN ] Bit map of type  chipcHw_REG_BUS_CLOCK_XXXXX */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_audioChannelEnable(uint32_t mask	/*  [ IN ] Bit map of type  chipcHw_REG_AUDIO_CHANNEL_XXXXXX */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_audioChannelDisable(uint32_t mask	/*  [ IN ] Bit map of type  chipcHw_REG_AUDIO_CHANNEL_XXXXXX */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_softReset(uint64_t mask	/*  [ IN ] Bit map of type chipcHw_REG_SOFT_RESET_XXXXXX */
    );

static inline void chipcHw_softResetDisable(uint64_t mask	/*  [ IN ] Bit map of type chipcHw_REG_SOFT_RESET_XXXXXX */
    );

static inline void chipcHw_softResetEnable(uint64_t mask	/*  [ IN ] Bit map of type chipcHw_REG_SOFT_RESET_XXXXXX */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_miscControl(uint32_t mask	/*  [ IN ] Bit map of type chipcHw_REG_MISC_CTRL_XXXXXX */
    );

static inline void chipcHw_miscControlDisable(uint32_t mask	/*  [ IN ] Bit map of type chipcHw_REG_MISC_CTRL_XXXXXX */
    );

static inline void chipcHw_miscControlEnable(uint32_t mask	/*  [ IN ] Bit map of type chipcHw_REG_MISC_CTRL_XXXXXX */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_setOTPOption(uint64_t mask	/*  [ IN ] Bit map of type chipcHw_REG_OTP_XXXXXX */
    );

/****************************************************************************/
/****************************************************************************/
static inline uint32_t chipcHw_getStickyBits(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_setStickyBits(uint32_t mask	/*  [ IN ] Bit map of type chipcHw_REG_STICKY_XXXXXX */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_clearStickyBits(uint32_t mask	/*  [ IN ] Bit map of type chipcHw_REG_STICKY_XXXXXX */
    );

/****************************************************************************/
/****************************************************************************/
static inline uint32_t chipcHw_getSoftStraps(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_setSoftStraps(uint32_t strapOptions);

/****************************************************************************/
/****************************************************************************/
static inline uint32_t chipcHw_getPinStraps(void);

/****************************************************************************/
/****************************************************************************/
static inline uint32_t chipcHw_getValidStraps(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_initValidStraps(void);

/****************************************************************************/
/****************************************************************************/
static inline uint32_t chipcHw_getBusInterfaceClockStatus(void);

/****************************************************************************/
/****************************************************************************/
static inline chipcHw_BOOT_DEVICE_e chipcHw_getBootDevice(void);

/****************************************************************************/
/****************************************************************************/
static inline chipcHw_BOOT_MODE_e chipcHw_getBootMode(void);

/****************************************************************************/
/****************************************************************************/
static inline chipcHw_NAND_PAGESIZE_e chipcHw_getNandPageSize(void);

/****************************************************************************/
/****************************************************************************/
static inline int chipcHw_getNandExtraCycle(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_activatePifInterface(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_activateLcdInterface(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_deactivatePifLcdInterface(void);

/****************************************************************************/
/****************************************************************************/
static inline chipcHw_GPIO_FUNCTION_e chipcHw_getGpioPinFunction(int pin	/* GPIO Pin number */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_setGpioPinFunction(int pin,	/* GPIO Pin number */
					      chipcHw_GPIO_FUNCTION_e func	/* Configuration function */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_setPinSlewRate(uint32_t pin,	/* Pin of type chipcHw_PIN_XXXXX */
					  chipcHw_PIN_SLEW_RATE_e slewRate	/* Pin slew rate */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_setPinOutputCurrent(uint32_t pin,	/* Pin of type chipcHw_PIN_XXXXX */
					       chipcHw_PIN_CURRENT_STRENGTH_e curr	/* Pin current rating */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_setPinPullup(uint32_t pin,	/* Pin of type chipcHw_PIN_XXXXX */
					chipcHw_PIN_PULL_e pullup	/* Pullup register settings */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_setPinInputType(uint32_t pin,	/* Pin of type chipcHw_PIN_XXXXX */
					   chipcHw_PIN_INPUTTYPE_e inputType	/* Pin input type */
    );

/****************************************************************************/
/****************************************************************************/

const char *chipcHw_getGpioPinFunctionStr(int pin);

/****************************************************************************/
/****************************************************************************/
void chipcHw_reset(uint32_t mask);

/****************************************************************************/
/****************************************************************************/
void chipcHw_clockReconfig(uint32_t busHz, uint32_t armRatio, uint32_t vpmRatio,
			   uint32_t ddrRatio);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_enableSpreadSpectrum(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_disableSpreadSpectrum(void);

/****************************************************************************/
/****************************************************************************/
static inline int chipcHw_isSoftwareStrapsEnable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_softwareStrapsEnable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_softwareStrapsDisable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_pllTestEnable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_pll2TestEnable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_pllTestDisable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_pll2TestDisable(void);

/****************************************************************************/
/****************************************************************************/
static inline int chipcHw_isPllTestEnable(void);

/****************************************************************************/
/****************************************************************************/
static inline int chipcHw_isPll2TestEnable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_pllTestSelect(uint32_t val);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_pll2TestSelect(uint32_t val);

/****************************************************************************/
/****************************************************************************/
static inline uint8_t chipcHw_getPllTestSelected(void);

/****************************************************************************/
/****************************************************************************/
static inline uint8_t chipcHw_getPll2TestSelected(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_ddrPhaseAlignInterruptEnable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_ddrPhaseAlignInterruptDisable(void);

/****************************************************************************/
/****************************************************************************/
static inline void
chipcHw_vpmPhaseAlignInterruptMode(chipcHw_VPM_HW_PHASE_INTR_e mode);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_ddrSwPhaseAlignEnable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_ddrSwPhaseAlignDisable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_ddrHwPhaseAlignEnable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_ddrHwPhaseAlignDisable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_vpmSwPhaseAlignEnable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_vpmSwPhaseAlignDisable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_vpmHwPhaseAlignEnable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_vpmHwPhaseAlignDisable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_setDdrHwPhaseAlignMargin(chipcHw_DDR_HW_PHASE_MARGIN_e margin	/* Margin alinging DDR  phase */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_setVpmHwPhaseAlignMargin(chipcHw_VPM_HW_PHASE_MARGIN_e margin	/* Margin alinging VPM  phase */
    );

/****************************************************************************/
/****************************************************************************/
static inline uint32_t chipcHw_isDdrHwPhaseAligned(void);

/****************************************************************************/
/****************************************************************************/
static inline uint32_t chipcHw_isVpmHwPhaseAligned(void);

/****************************************************************************/
/****************************************************************************/
static inline uint32_t chipcHw_getDdrHwPhaseAlignStatus(void);

/****************************************************************************/
/****************************************************************************/
static inline uint32_t chipcHw_getVpmHwPhaseAlignStatus(void);

/****************************************************************************/
/****************************************************************************/
static inline uint32_t chipcHw_getDdrPhaseControl(void);

/****************************************************************************/
/****************************************************************************/
static inline uint32_t chipcHw_getVpmPhaseControl(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_ddrHwPhaseAlignTimeout(uint32_t busCycle	/* Timeout in bus cycle */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_vpmHwPhaseAlignTimeout(uint32_t busCycle	/* Timeout in bus cycle */
    );

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_ddrHwPhaseAlignTimeoutInterruptEnable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_vpmHwPhaseAlignTimeoutInterruptEnable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_ddrHwPhaseAlignTimeoutInterruptDisable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_vpmHwPhaseAlignTimeoutInterruptDisable(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_ddrHwPhaseAlignTimeoutInterruptClear(void);

/****************************************************************************/
/****************************************************************************/
static inline void chipcHw_vpmHwPhaseAlignTimeoutInterruptClear(void);

/* ---- Private Constants and Types -------------------------------------- */

#endif /* CHIPC_DEF_H */
