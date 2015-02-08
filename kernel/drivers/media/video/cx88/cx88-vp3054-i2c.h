

/* ----------------------------------------------------------------------- */
struct vp3054_i2c_state {
	struct i2c_adapter         adap;
	struct i2c_algo_bit_data   algo;
	u32                        state;
};

/* ----------------------------------------------------------------------- */
#if defined(CONFIG_VIDEO_CX88_VP3054) || (defined(CONFIG_VIDEO_CX88_VP3054_MODULE) && defined(MODULE))
int  vp3054_i2c_probe(struct cx8802_dev *dev);
void vp3054_i2c_remove(struct cx8802_dev *dev);
#else
static inline int  vp3054_i2c_probe(struct cx8802_dev *dev)
{ return 0; }
static inline void vp3054_i2c_remove(struct cx8802_dev *dev)
{ }
#endif
