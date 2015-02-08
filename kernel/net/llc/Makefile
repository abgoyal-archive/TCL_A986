obj-$(CONFIG_LLC) += llc.o

llc-y := llc_core.o llc_input.o llc_output.o

obj-$(CONFIG_LLC2) += llc2.o

llc2-y := llc_if.o llc_c_ev.o llc_c_ac.o llc_conn.o llc_c_st.o llc_pdu.o \
	  llc_sap.o llc_s_ac.o llc_s_ev.o llc_s_st.o af_llc.o llc_station.o

llc2-$(CONFIG_PROC_FS) += llc_proc.o
llc2-$(CONFIG_SYSCTL)  += sysctl_net_llc.o
