

#ifndef _IXGBE_SRIOV_H_
#define _IXGBE_SRIOV_H_

int ixgbe_set_vf_multicasts(struct ixgbe_adapter *adapter,
                            int entries, u16 *hash_list, u32 vf);
void ixgbe_restore_vf_multicasts(struct ixgbe_adapter *adapter);
int ixgbe_set_vf_vlan(struct ixgbe_adapter *adapter, int add, int vid, u32 vf);
void ixgbe_set_vmolr(struct ixgbe_hw *hw, u32 vf, bool aupe);
void ixgbe_vf_reset_event(struct ixgbe_adapter *adapter, u32 vf);
void ixgbe_vf_reset_msg(struct ixgbe_adapter *adapter, u32 vf);
void ixgbe_msg_task(struct ixgbe_adapter *adapter);
int ixgbe_set_vf_mac(struct ixgbe_adapter *adapter,
                     int vf, unsigned char *mac_addr);
int ixgbe_vf_configuration(struct pci_dev *pdev, unsigned int event_mask);
void ixgbe_disable_tx_rx(struct ixgbe_adapter *adapter);
void ixgbe_ping_all_vfs(struct ixgbe_adapter *adapter);
void ixgbe_dump_registers(struct ixgbe_adapter *adapter);
int ixgbe_ndo_set_vf_mac(struct net_device *netdev, int queue, u8 *mac);
int ixgbe_ndo_set_vf_vlan(struct net_device *netdev, int queue, u16 vlan,
			   u8 qos);
int ixgbe_ndo_set_vf_bw(struct net_device *netdev, int vf, int tx_rate);
int ixgbe_ndo_get_vf_config(struct net_device *netdev,
			    int vf, struct ifla_vf_info *ivi);

#endif /* _IXGBE_SRIOV_H_ */

