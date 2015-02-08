

#ifndef _MSP3400_H_
#define _MSP3400_H_



/* SCART input to DSP selection */
#define MSP_IN_SCART1  		0  /* Pin SC1_IN */
#define MSP_IN_SCART2  		1  /* Pin SC2_IN */
#define MSP_IN_SCART3  		2  /* Pin SC3_IN */
#define MSP_IN_SCART4  		3  /* Pin SC4_IN */
#define MSP_IN_MONO     	6  /* Pin MONO_IN */
#define MSP_IN_MUTE     	7  /* Mute DSP input */
#define MSP_SCART_TO_DSP(in) 	(in)
/* Tuner input to demodulator and DSP selection */
#define MSP_IN_TUNER1 		0  /* Analog Sound IF input pin ANA_IN1 */
#define MSP_IN_TUNER2 		1  /* Analog Sound IF input pin ANA_IN2 */
#define MSP_TUNER_TO_DSP(in) 	((in) << 3)

#define MSP_DSP_IN_TUNER 	0  /* Tuner DSP input */
#define MSP_DSP_IN_SCART 	2  /* SCART DSP input */
#define MSP_DSP_IN_I2S1 	5  /* I2S1 DSP input */
#define MSP_DSP_IN_I2S2 	6  /* I2S2 DSP input */
#define MSP_DSP_IN_I2S3    	7  /* I2S3 DSP input */
#define MSP_DSP_IN_MAIN_AVC 	11 /* MAIN AVC processed DSP input */
#define MSP_DSP_IN_MAIN 	12 /* MAIN DSP input */
#define MSP_DSP_IN_AUX 		13 /* AUX DSP input */
#define MSP_DSP_TO_MAIN(in)   	((in) << 4)
#define MSP_DSP_TO_AUX(in)    	((in) << 8)
#define MSP_DSP_TO_SCART1(in) 	((in) << 12)
#define MSP_DSP_TO_SCART2(in) 	((in) << 16)
#define MSP_DSP_TO_I2S(in)    	((in) << 20)

#define MSP_SC_IN_SCART1 	0  /* SCART1 input, bypassing the DSP */
#define MSP_SC_IN_SCART2 	1  /* SCART2 input, bypassing the DSP */
#define MSP_SC_IN_SCART3 	2  /* SCART3 input, bypassing the DSP */
#define MSP_SC_IN_SCART4 	3  /* SCART4 input, bypassing the DSP */
#define MSP_SC_IN_DSP_SCART1 	4  /* DSP SCART1 input */
#define MSP_SC_IN_DSP_SCART2 	5  /* DSP SCART2 input */
#define MSP_SC_IN_MONO 		6  /* MONO input, bypassing the DSP */
#define MSP_SC_IN_MUTE 		7  /* MUTE output */
#define MSP_SC_TO_SCART1(in)	(in)
#define MSP_SC_TO_SCART2(in)	((in) << 4)

/* Shortcut macros */
#define MSP_INPUT(sc, t, main_aux_src, sc_i2s_src) \
	(MSP_SCART_TO_DSP(sc) | \
	 MSP_TUNER_TO_DSP(t) | \
	 MSP_DSP_TO_MAIN(main_aux_src) | \
	 MSP_DSP_TO_AUX(main_aux_src) | \
	 MSP_DSP_TO_SCART1(sc_i2s_src) | \
	 MSP_DSP_TO_SCART2(sc_i2s_src) | \
	 MSP_DSP_TO_I2S(sc_i2s_src))
#define MSP_INPUT_DEFAULT MSP_INPUT(MSP_IN_SCART1, MSP_IN_TUNER1, \
				    MSP_DSP_IN_TUNER, MSP_DSP_IN_TUNER)
#define MSP_OUTPUT(sc) \
	(MSP_SC_TO_SCART1(sc) | \
	 MSP_SC_TO_SCART2(sc))
/* This equals the RESET position of the msp3400 ACB register */
#define MSP_OUTPUT_DEFAULT (MSP_SC_TO_SCART1(MSP_SC_IN_SCART3) | \
			    MSP_SC_TO_SCART2(MSP_SC_IN_DSP_SCART1))

/* Tuner inputs vs. msp version */

/* SCART inputs vs. msp version */

/* DSP inputs vs. msp version (tuner and SCART inputs are always available) */

/* DSP outputs vs. msp version */

#endif /* MSP3400_H */

