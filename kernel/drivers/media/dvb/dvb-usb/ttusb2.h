
#ifndef _DVB_USB_TTUSB2_H_
#define _DVB_USB_TTUSB2_H_


#define CMD_DSP_DOWNLOAD    0x13

#define CMD_DSP_BOOT        0x14
/* out data: nothing */

#define CMD_POWER           0x15
/* out data: <on=1/off=0> */

#define CMD_LNB             0x16
/* out data: <power=1> <18V=0,13V=1> <tone> <??=1> <??=1> */

#define CMD_GET_VERSION     0x17
/* in  data: <version_byte>[5] */

#define CMD_DISEQC          0x18
/* out data: <master=0xff/burst=??> <cmdlen> <cmdbytes>[cmdlen] */

#define CMD_PID_ENABLE      0x22
/* out data: <index> <type: ts=1/sec=2> <pid msb> <pid lsb> */

#define CMD_PID_DISABLE     0x23
/* out data: <index> */

#define CMD_FILTER_ENABLE   0x24
/* out data: <index> <pid_idx> <filter>[12] <mask>[12] */

#define CMD_FILTER_DISABLE  0x25
/* out data: <index> */

#define CMD_GET_DSP_VERSION 0x26
/* in  data: <version_byte>[28] */

#define CMD_I2C_XFER        0x31

#define CMD_I2C_BITRATE     0x32
/* out data: <default=0> */

#endif
