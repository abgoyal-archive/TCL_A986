

#define S3C64XX_AC97_GPD  0
#define S3C64XX_AC97_GPE  1
extern void s3c64xx_ac97_setup_gpio(int);

struct s3c_audio_pdata {
	int (*cfg_gpio)(struct platform_device *);
};
