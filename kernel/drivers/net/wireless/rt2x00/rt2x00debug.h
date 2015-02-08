


#ifndef RT2X00DEBUG_H
#define RT2X00DEBUG_H

struct rt2x00_dev;

enum rt2x00debugfs_entry_flags {
	RT2X00DEBUGFS_OFFSET	= (1 << 0),
};

#define RT2X00DEBUGFS_REGISTER_ENTRY(__name, __type)		\
struct reg##__name {						\
	void (*read)(struct rt2x00_dev *rt2x00dev,		\
		     const unsigned int word, __type *data);	\
	void (*write)(struct rt2x00_dev *rt2x00dev,		\
		      const unsigned int word, __type data);	\
								\
	unsigned int flags;					\
								\
	unsigned int word_base;					\
	unsigned int word_size;					\
	unsigned int word_count;				\
} __name

struct rt2x00debug {
	/*
	 * Reference to the modules structure.
	 */
	struct module *owner;

	/*
	 * Register access entries.
	 */
	RT2X00DEBUGFS_REGISTER_ENTRY(csr, u32);
	RT2X00DEBUGFS_REGISTER_ENTRY(eeprom, u16);
	RT2X00DEBUGFS_REGISTER_ENTRY(bbp, u8);
	RT2X00DEBUGFS_REGISTER_ENTRY(rf, u32);
};

#endif /* RT2X00DEBUG_H */
