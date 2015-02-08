

#ifndef RTL8225H
#define RTL8225H

#ifdef RTL8190P
#define RTL819X_TOTAL_RF_PATH 4 //for 90P
#else
#define RTL819X_TOTAL_RF_PATH 2 //for 8192U
#endif
extern void PHY_SetRF8256Bandwidth(struct net_device* dev , HT_CHANNEL_WIDTH Bandwidth);
extern void PHY_RF8256_Config(struct net_device* dev);
extern void phy_RF8256_Config_ParaFile(struct net_device* dev);
extern void PHY_SetRF8256CCKTxPower(struct net_device*	dev, u8	powerlevel);
extern void PHY_SetRF8256OFDMTxPower(struct net_device* dev, u8 powerlevel);

#endif
