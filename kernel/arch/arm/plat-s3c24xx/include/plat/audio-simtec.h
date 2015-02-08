

struct s3c24xx_audio_simtec_pdata {
	unsigned int	use_mpllin:1;
	unsigned int	output_cdclk:1;

	unsigned int	have_mic:1;
	unsigned int	have_lout:1;

	int		amp_gpio;
	int		amp_gain[2];

	void	(*startup)(void);
};

extern int simtec_audio_add(const char *codec_name, bool has_lr_routing,
			    struct s3c24xx_audio_simtec_pdata *pdata);
