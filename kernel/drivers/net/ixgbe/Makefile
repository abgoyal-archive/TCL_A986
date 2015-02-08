obj-$(CONFIG_IXGBE) += ixgbe.o

ixgbe-objs := ixgbe_main.o ixgbe_common.o ixgbe_ethtool.o \
              ixgbe_82599.o ixgbe_82598.o ixgbe_phy.o ixgbe_sriov.o \
              ixgbe_mbx.o

ixgbe-$(CONFIG_IXGBE_DCB) +=  ixgbe_dcb.o ixgbe_dcb_82598.o \
                              ixgbe_dcb_82599.o ixgbe_dcb_nl.o

ixgbe-$(CONFIG_FCOE:m=y) += ixgbe_fcoe.o
