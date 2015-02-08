
#ifndef QDSP5LPMCMDI_H
#define QDSP5LPMCMDI_H




#define	LPM_CMD_START		0x0000
#define	LPM_CMD_START_LEN	sizeof(lpm_cmd_start)

#define	LPM_CMD_SPATIAL_FILTER_PART_OPMODE_0	0x00000000
#define	LPM_CMD_SPATIAL_FILTER_PART_OPMODE_1	0x00010000
typedef struct {
	unsigned int	cmd_id;
	unsigned int	ip_data_cfg_part1;
	unsigned int	ip_data_cfg_part2;
	unsigned int	ip_data_cfg_part3;
	unsigned int	ip_data_cfg_part4;
	unsigned int	op_data_cfg_part1;
	unsigned int	op_data_cfg_part2;
	unsigned int	op_data_cfg_part3;
	unsigned int	spatial_filter_part[32];
} __attribute__((packed)) lpm_cmd_start;




#define	LPM_CMD_IDLE		0x0001
#define	LPM_CMD_IDLE_LEN	sizeof(lpm_cmd_idle)

typedef struct {
	unsigned int	cmd_id;
} __attribute__((packed)) lpm_cmd_idle;


#endif
