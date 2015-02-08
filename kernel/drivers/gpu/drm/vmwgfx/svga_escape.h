


#ifndef _SVGA_ESCAPE_H_
#define _SVGA_ESCAPE_H_



#define SVGA_ESCAPE_NSID_VMWARE 0x00000000
#define SVGA_ESCAPE_NSID_DEVEL  0xFFFFFFFF



#define SVGA_ESCAPE_VMWARE_MAJOR_MASK  0xFFFF0000



#define SVGA_ESCAPE_VMWARE_HINT               0x00030000
#define SVGA_ESCAPE_VMWARE_HINT_FULLSCREEN    0x00030001  // Deprecated

typedef
struct {
   uint32 command;
   uint32 fullscreen;
   struct {
      int32 x, y;
   } monitorPosition;
} SVGAEscapeHintFullscreen;

#endif /* _SVGA_ESCAPE_H_ */
