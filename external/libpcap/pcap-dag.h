

pcap_t *dag_open_live(const char *device, int snaplen, int promisc, int to_ms, char *ebuf);
int dag_platform_finddevs(pcap_if_t **devlistp, char *errbuf);
