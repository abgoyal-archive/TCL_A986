

#ifndef __HISAX_DEBUG_H__
#define __HISAX_DEBUG_H__


#ifdef CONFIG_HISAX_DEBUG

#define DBG(level, format, arg...) do { \
if (level & __debug_variable) \
printk(KERN_DEBUG "%s: " format "\n" , __func__ , ## arg); \
} while (0)

#define DBG_PACKET(level,data,count) \
  if (level & __debug_variable) dump_packet(__func__,data,count)

#define DBG_SKB(level,skb) \
  if ((level & __debug_variable) && skb) dump_packet(__func__,skb->data,skb->len)


static void __attribute__((unused))
dump_packet(const char *name,const u_char *data,int pkt_len)
{
#define DUMP_HDR_SIZE 20
#define DUMP_TLR_SIZE 8
	if (pkt_len) {
		int i,len1,len2;

		printk(KERN_DEBUG "%s: length=%d,data=",name,pkt_len);

		if (pkt_len >  DUMP_HDR_SIZE+ DUMP_TLR_SIZE) {
			len1 = DUMP_HDR_SIZE;
			len2 = DUMP_TLR_SIZE;
		} else {
			len1 = pkt_len > DUMP_HDR_SIZE ? DUMP_HDR_SIZE : pkt_len;
			len2 = 0;			
		}
		for (i = 0; i < len1; ++i) {
		 	printk ("%.2x", data[i]);
		}
		if (len2) {
		 	printk ("..");
			for (i = pkt_len-DUMP_TLR_SIZE; i < pkt_len; ++i) {
				printk ("%.2x", data[i]);
			}
		}
		printk ("\n");
	}
#undef DUMP_HDR_SIZE
#undef DUMP_TLR_SIZE
}

#else

#define DBG(level, format, arg...) do {} while (0)
#define DBG_PACKET(level,data,count) do {} while (0)
#define DBG_SKB(level,skb) do {} while (0)

#endif

#endif
