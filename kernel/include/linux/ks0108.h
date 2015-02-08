

#ifndef _KS0108_H_
#define _KS0108_H_

/* Write a byte to the data port */
extern void ks0108_writedata(unsigned char byte);

/* Write a byte to the control port */
extern void ks0108_writecontrol(unsigned char byte);

/* Set the controller's current display state (0..1) */
extern void ks0108_displaystate(unsigned char state);

/* Set the controller's current startline (0..63) */
extern void ks0108_startline(unsigned char startline);

/* Set the controller's current address (0..63) */
extern void ks0108_address(unsigned char address);

/* Set the controller's current page (0..7) */
extern void ks0108_page(unsigned char page);

/* Is the module inited? */
extern unsigned char ks0108_isinited(void);

#endif /* _KS0108_H_ */
