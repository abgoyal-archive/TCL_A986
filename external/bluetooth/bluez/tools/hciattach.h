
#include <termios.h>

#ifndef N_HCI
#define N_HCI	15
#endif

#define HCIUARTSETPROTO		_IOW('U', 200, int)
#define HCIUARTGETPROTO		_IOR('U', 201, int)
#define HCIUARTGETDEVICE	_IOR('U', 202, int)
#define HCIUARTSETFLAGS		_IOW('U', 203, int)
#define HCIUARTGETFLAGS		_IOR('U', 204, int)

#define HCI_UART_H4	0
#define HCI_UART_BCSP	1
#define HCI_UART_3WIRE	2
#define HCI_UART_H4DS	3
#define HCI_UART_LL	4

#define HCI_UART_RAW_DEVICE	0

int read_hci_event(int fd, unsigned char* buf, int size);
int set_speed(int fd, struct termios *ti, int speed);

int texas_init(int fd, struct termios *ti);
int texas_post(int fd, struct termios *ti);
int texasalt_init(int fd, int speed, struct termios *ti);
int stlc2500_init(int fd, bdaddr_t *bdaddr);
int bgb2xx_init(int dd, bdaddr_t *bdaddr);
