


extern void (*vp8_clear_system_state)(void);
extern void (*vp8_plane_add_noise)(unsigned char *Start, unsigned int Width, unsigned int Height, int Pitch, int DPitch, int q);
extern void (*de_interlace)
(
    unsigned char *src_ptr,
    unsigned char *dst_ptr,
    int Width,
    int Height,
    int Stride
);
