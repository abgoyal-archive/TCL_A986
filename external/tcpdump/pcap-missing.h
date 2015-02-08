

#ifndef tcpdump_pcap_missing_h
#define tcpdump_pcap_missing_h


#ifndef HAVE_PCAP_LIST_DATALINKS
extern int pcap_list_datalinks(pcap_t *, int **);
#endif

#ifndef HAVE_PCAP_DATALINK_NAME_TO_VAL
extern int pcap_datalink_name_to_val(const char *);
extern const char *pcap_datalink_val_to_name(int);
#endif

#ifndef HAVE_PCAP_DATALINK_VAL_TO_DESCRIPTION
extern const char *pcap_datalink_val_to_description(int);
#endif

#ifndef HAVE_PCAP_DUMP_FTELL
extern long pcap_dump_ftell(pcap_dumper_t *);
#endif

#endif









