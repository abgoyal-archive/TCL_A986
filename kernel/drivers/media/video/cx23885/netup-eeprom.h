

#ifndef NETUP_EEPROM_H
#define NETUP_EEPROM_H

struct netup_port_info {
	u8 mac[6];/* card MAC address */
};

struct netup_card_info {
	struct netup_port_info port[2];/* ports - 1,2 */
	u8 rev;/* card revision */
};

extern int netup_eeprom_read(struct i2c_adapter *i2c_adap, u8 addr);
extern int netup_eeprom_write(struct i2c_adapter *i2c_adap, u8 addr, u8 data);
extern void netup_get_card_info(struct i2c_adapter *i2c_adap,
				struct netup_card_info *cinfo);

#endif
