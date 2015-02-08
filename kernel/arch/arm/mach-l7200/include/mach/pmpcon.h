
/****************************************************************************/

/****************************************************************************/

#define PMPCON_OFF 0x00006000  /* Offset from IO_START_2. */

/* IO_START_2 and IO_BASE_2 are defined in hardware.h */

#define PMPCON_START (IO_START_2 + PMPCON_OFF)  /* Physical address of reg. */
#define PMPCON_BASE  (IO_BASE_2  + PMPCON_OFF)  /* Virtual address of reg. */


#define PMPCON (*(volatile unsigned int *)(PMPCON_BASE))

#define PWM2_50CYCLE 0x800
#define CONTRAST     0x9

#define PWM1H (CONTRAST)
#define PWM1L (CONTRAST << 4)

#define PMPCON_VALUE  (PWM2_50CYCLE | PWM1L | PWM1H) 
	

