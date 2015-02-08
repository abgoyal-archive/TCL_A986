

#ifndef RTL8225H
#define RTL8225H

#ifdef RTL8190P
#define RTL819X_TOTAL_RF_PATH 4 //for 90P
#else
#define RTL819X_TOTAL_RF_PATH 2 //for 8192U
#endif
extern void PHY_SetRF0222DBandwidth(struct net_device* dev , HT_CHANNEL_WIDTH Bandwidth);	//20M or 40M;
extern void PHY_SetRF8225Bandwidth(	struct net_device* dev ,	HT_CHANNEL_WIDTH Bandwidth);
extern bool PHY_RF8225_Config(struct net_device* dev );
extern void phy_RF8225_Config_HardCode(struct net_device*	dev);
extern bool phy_RF8225_Config_ParaFile(struct net_device* dev);
extern void PHY_SetRF8225CckTxPower(struct net_device* dev ,u8 powerlevel);
extern void PHY_SetRF8225OfdmTxPower(struct net_device* dev ,u8        powerlevel);
extern void PHY_SetRF0222DOfdmTxPower(struct net_device* dev ,u8 powerlevel);
extern void PHY_SetRF0222DCckTxPower(struct net_device* dev ,u8        powerlevel);
#endif
