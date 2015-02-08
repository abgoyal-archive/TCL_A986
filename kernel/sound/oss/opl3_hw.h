


#define TEST_REGISTER				0x01
#define   ENABLE_WAVE_SELECT		0x20

#define TIMER1_REGISTER				0x02
#define TIMER2_REGISTER				0x03
#define TIMER_CONTROL_REGISTER			0x04	/* Left side */
#define   IRQ_RESET			0x80
#define   TIMER1_MASK			0x40
#define   TIMER2_MASK			0x20
#define   TIMER1_START			0x01
#define   TIMER2_START			0x02

#define CONNECTION_SELECT_REGISTER		0x04	/* Right side */
#define   RIGHT_4OP_0			0x01
#define   RIGHT_4OP_1			0x02
#define   RIGHT_4OP_2			0x04
#define   LEFT_4OP_0			0x08
#define   LEFT_4OP_1			0x10
#define   LEFT_4OP_2			0x20

#define OPL3_MODE_REGISTER			0x05	/* Right side */
#define   OPL3_ENABLE			0x01
#define   OPL4_ENABLE			0x02

#define KBD_SPLIT_REGISTER			0x08	/* Left side */
#define   COMPOSITE_SINE_WAVE_MODE	0x80		/* Don't use with OPL-3? */
#define   KEYBOARD_SPLIT		0x40

#define PERCOSSION_REGISTER			0xbd	/* Left side only */
#define   TREMOLO_DEPTH			0x80
#define   VIBRATO_DEPTH			0x40
#define	  PERCOSSION_ENABLE		0x20
#define   BASSDRUM_ON			0x10
#define   SNAREDRUM_ON			0x08
#define   TOMTOM_ON			0x04
#define   CYMBAL_ON			0x02
#define   HIHAT_ON			0x01

#define AM_VIB					0x20
#define   TREMOLO_ON			0x80
#define   VIBRATO_ON			0x40
#define   SUSTAIN_ON			0x20
#define   KSR				0x10 	/* Key scaling rate */
#define   MULTIPLE_MASK		0x0f	/* Frequency multiplier */

 /*
  *	KSL/Total level (0x40 to 0x55)
  */
#define KSL_LEVEL				0x40
#define   KSL_MASK			0xc0	/* Envelope scaling bits */
#define   TOTAL_LEVEL_MASK		0x3f	/* Strength (volume) of OP */

#define ATTACK_DECAY				0x60
#define   ATTACK_MASK			0xf0
#define   DECAY_MASK			0x0f

#define SUSTAIN_RELEASE				0x80
#define   SUSTAIN_MASK			0xf0
#define   RELEASE_MASK			0x0f

#define WAVE_SELECT			0xe0

#define FNUM_LOW				0xa0

#define KEYON_BLOCK					0xb0
#define	  KEYON_BIT				0x20
#define	  BLOCKNUM_MASK				0x1c
#define   FNUM_HIGH_MASK			0x03

#define FEEDBACK_CONNECTION				0xc0
#define   FEEDBACK_MASK				0x0e	/* Valid just for 1st OP of a voice */
#define   CONNECTION_BIT			0x01
#define   STEREO_BITS				0x30	/* OPL-3 only */
#define     VOICE_TO_LEFT		0x10
#define     VOICE_TO_RIGHT		0x20


struct physical_voice_info {
		unsigned char voice_num;
		unsigned char voice_mode; /* 0=unavailable, 2=2 OP, 4=4 OP */
		unsigned short ioaddr; /* I/O port (left or right side) */
		unsigned char op[4]; /* Operator offsets */
	};


#define USE_LEFT	0
#define USE_RIGHT	1

static struct physical_voice_info pv_map[18] =
{
/*       No Mode Side		OP1	OP2	OP3   OP4	*/
/*	---------------------------------------------------	*/
	{ 0,  2, USE_LEFT,	{0x00,	0x03,	0x08, 0x0b}},
	{ 1,  2, USE_LEFT,	{0x01,	0x04,	0x09, 0x0c}},
	{ 2,  2, USE_LEFT,	{0x02,	0x05,	0x0a, 0x0d}},

	{ 3,  2, USE_LEFT,	{0x08,	0x0b,	0x00, 0x00}},
	{ 4,  2, USE_LEFT,	{0x09,	0x0c,	0x00, 0x00}},
	{ 5,  2, USE_LEFT,	{0x0a,	0x0d,	0x00, 0x00}},

	{ 6,  2, USE_LEFT,	{0x10,	0x13,	0x00, 0x00}}, /* Used by percussive voices */
	{ 7,  2, USE_LEFT,	{0x11,	0x14,	0x00, 0x00}}, /* if the percussive mode */
	{ 8,  2, USE_LEFT,	{0x12,	0x15,	0x00, 0x00}}, /* is selected */

	{ 0,  2, USE_RIGHT,	{0x00,	0x03,	0x08, 0x0b}},
	{ 1,  2, USE_RIGHT,	{0x01,	0x04,	0x09, 0x0c}},
	{ 2,  2, USE_RIGHT,	{0x02,	0x05,	0x0a, 0x0d}},

	{ 3,  2, USE_RIGHT,	{0x08,	0x0b,	0x00, 0x00}},
	{ 4,  2, USE_RIGHT,	{0x09,	0x0c,	0x00, 0x00}},
	{ 5,  2, USE_RIGHT,	{0x0a,	0x0d,	0x00, 0x00}},

	{ 6,  2, USE_RIGHT,	{0x10,	0x13,	0x00, 0x00}},
	{ 7,  2, USE_RIGHT,	{0x11,	0x14,	0x00, 0x00}},
	{ 8,  2, USE_RIGHT,	{0x12,	0x15,	0x00, 0x00}}
};
