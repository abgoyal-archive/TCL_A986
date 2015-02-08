


#ifndef _SVGA_REG_H_
#define _SVGA_REG_H_

#define PCI_VENDOR_ID_VMWARE            0x15AD
#define PCI_DEVICE_ID_VMWARE_SVGA2      0x0405

#define SVGA_CURSOR_ON_HIDE            0x0   /* Must be 0 to maintain backward compatibility */
#define SVGA_CURSOR_ON_SHOW            0x1   /* Must be 1 to maintain backward compatibility */
#define SVGA_CURSOR_ON_REMOVE_FROM_FB  0x2   /* Remove the cursor from the framebuffer because we need to see what's under it */
#define SVGA_CURSOR_ON_RESTORE_TO_FB   0x3   /* Put the cursor back in the framebuffer so the user can see it */

#define SVGA_FB_MAX_TRACEABLE_SIZE      0x1000000

#define SVGA_MAX_PSEUDOCOLOR_DEPTH      8
#define SVGA_MAX_PSEUDOCOLORS           (1 << SVGA_MAX_PSEUDOCOLOR_DEPTH)
#define SVGA_NUM_PALETTE_REGS           (3 * SVGA_MAX_PSEUDOCOLORS)

#define SVGA_MAGIC         0x900000UL
#define SVGA_MAKE_ID(ver)  (SVGA_MAGIC << 8 | (ver))

/* Version 2 let the address of the frame buffer be unsigned on Win32 */
#define SVGA_VERSION_2     2
#define SVGA_ID_2          SVGA_MAKE_ID(SVGA_VERSION_2)

#define SVGA_VERSION_1     1
#define SVGA_ID_1          SVGA_MAKE_ID(SVGA_VERSION_1)

/* Version 0 is the initial version */
#define SVGA_VERSION_0     0
#define SVGA_ID_0          SVGA_MAKE_ID(SVGA_VERSION_0)

/* "Invalid" value for all SVGA IDs. (Version ID, screen object ID, surface ID...) */
#define SVGA_ID_INVALID    0xFFFFFFFF

/* Port offsets, relative to BAR0 */
#define SVGA_INDEX_PORT         0x0
#define SVGA_VALUE_PORT         0x1
#define SVGA_BIOS_PORT          0x2
#define SVGA_IRQSTATUS_PORT     0x8

#define SVGA_IRQFLAG_ANY_FENCE            0x1    /* Any fence was passed */
#define SVGA_IRQFLAG_FIFO_PROGRESS        0x2    /* Made forward progress in the FIFO */
#define SVGA_IRQFLAG_FENCE_GOAL           0x4    /* SVGA_FIFO_FENCE_GOAL reached */


enum {
   SVGA_REG_ID = 0,
   SVGA_REG_ENABLE = 1,
   SVGA_REG_WIDTH = 2,
   SVGA_REG_HEIGHT = 3,
   SVGA_REG_MAX_WIDTH = 4,
   SVGA_REG_MAX_HEIGHT = 5,
   SVGA_REG_DEPTH = 6,
   SVGA_REG_BITS_PER_PIXEL = 7,       /* Current bpp in the guest */
   SVGA_REG_PSEUDOCOLOR = 8,
   SVGA_REG_RED_MASK = 9,
   SVGA_REG_GREEN_MASK = 10,
   SVGA_REG_BLUE_MASK = 11,
   SVGA_REG_BYTES_PER_LINE = 12,
   SVGA_REG_FB_START = 13,            /* (Deprecated) */
   SVGA_REG_FB_OFFSET = 14,
   SVGA_REG_VRAM_SIZE = 15,
   SVGA_REG_FB_SIZE = 16,

   /* ID 0 implementation only had the above registers, then the palette */

   SVGA_REG_CAPABILITIES = 17,
   SVGA_REG_MEM_START = 18,           /* (Deprecated) */
   SVGA_REG_MEM_SIZE = 19,
   SVGA_REG_CONFIG_DONE = 20,         /* Set when memory area configured */
   SVGA_REG_SYNC = 21,                /* See "FIFO Synchronization Registers" */
   SVGA_REG_BUSY = 22,                /* See "FIFO Synchronization Registers" */
   SVGA_REG_GUEST_ID = 23,            /* Set guest OS identifier */
   SVGA_REG_CURSOR_ID = 24,           /* (Deprecated) */
   SVGA_REG_CURSOR_X = 25,            /* (Deprecated) */
   SVGA_REG_CURSOR_Y = 26,            /* (Deprecated) */
   SVGA_REG_CURSOR_ON = 27,           /* (Deprecated) */
   SVGA_REG_HOST_BITS_PER_PIXEL = 28, /* (Deprecated) */
   SVGA_REG_SCRATCH_SIZE = 29,        /* Number of scratch registers */
   SVGA_REG_MEM_REGS = 30,            /* Number of FIFO registers */
   SVGA_REG_NUM_DISPLAYS = 31,        /* (Deprecated) */
   SVGA_REG_PITCHLOCK = 32,           /* Fixed pitch for all modes */
   SVGA_REG_IRQMASK = 33,             /* Interrupt mask */

   /* Legacy multi-monitor support */
   SVGA_REG_NUM_GUEST_DISPLAYS = 34,/* Number of guest displays in X/Y direction */
   SVGA_REG_DISPLAY_ID = 35,        /* Display ID for the following display attributes */
   SVGA_REG_DISPLAY_IS_PRIMARY = 36,/* Whether this is a primary display */
   SVGA_REG_DISPLAY_POSITION_X = 37,/* The display position x */
   SVGA_REG_DISPLAY_POSITION_Y = 38,/* The display position y */
   SVGA_REG_DISPLAY_WIDTH = 39,     /* The display's width */
   SVGA_REG_DISPLAY_HEIGHT = 40,    /* The display's height */

   /* See "Guest memory regions" below. */
   SVGA_REG_GMR_ID = 41,
   SVGA_REG_GMR_DESCRIPTOR = 42,
   SVGA_REG_GMR_MAX_IDS = 43,
   SVGA_REG_GMR_MAX_DESCRIPTOR_LENGTH = 44,

   SVGA_REG_TRACES = 45,            /* Enable trace-based updates even when FIFO is on */
   SVGA_REG_TOP = 46,               /* Must be 1 more than the last register */

   SVGA_PALETTE_BASE = 1024,        /* Base of SVGA color map */
   /* Next 768 (== 256*3) registers exist for colormap */

   SVGA_SCRATCH_BASE = SVGA_PALETTE_BASE + SVGA_NUM_PALETTE_REGS
                                    /* Base of scratch registers */
   /* Next reg[SVGA_REG_SCRATCH_SIZE] registers exist for scratch usage:
      First 4 are reserved for VESA BIOS Extension; any remaining are for
      the use of the current SVGA driver. */
};



#define SVGA_GMR_NULL         ((uint32) -1)
#define SVGA_GMR_FRAMEBUFFER  ((uint32) -2)  // Guest Framebuffer (GFB)

typedef
struct SVGAGuestMemDescriptor {
   uint32 ppn;
   uint32 numPages;
} SVGAGuestMemDescriptor;

typedef
struct SVGAGuestPtr {
   uint32 gmrId;
   uint32 offset;
} SVGAGuestPtr;



typedef
struct SVGAGMRImageFormat {
   union {
      struct {
         uint32 bitsPerPixel : 8;
         uint32 colorDepth   : 8;
         uint32 reserved     : 16;  // Must be zero
      };

      uint32 value;
   };
} SVGAGMRImageFormat;


typedef
struct SVGAColorBGRX {
   union {
      struct {
         uint32 b : 8;
         uint32 g : 8;
         uint32 r : 8;
         uint32 x : 8;  // Unused
      };

      uint32 value;
   };
} SVGAColorBGRX;



typedef
struct SVGASignedRect {
   int32  left;
   int32  top;
   int32  right;
   int32  bottom;
} SVGASignedRect;

typedef
struct SVGASignedPoint {
   int32  x;
   int32  y;
} SVGASignedPoint;



#define SVGA_CAP_NONE               0x00000000
#define SVGA_CAP_RECT_COPY          0x00000002
#define SVGA_CAP_CURSOR             0x00000020
#define SVGA_CAP_CURSOR_BYPASS      0x00000040   // Legacy (Use Cursor Bypass 3 instead)
#define SVGA_CAP_CURSOR_BYPASS_2    0x00000080   // Legacy (Use Cursor Bypass 3 instead)
#define SVGA_CAP_8BIT_EMULATION     0x00000100
#define SVGA_CAP_ALPHA_CURSOR       0x00000200
#define SVGA_CAP_3D                 0x00004000
#define SVGA_CAP_EXTENDED_FIFO      0x00008000
#define SVGA_CAP_MULTIMON           0x00010000   // Legacy multi-monitor support
#define SVGA_CAP_PITCHLOCK          0x00020000
#define SVGA_CAP_IRQMASK            0x00040000
#define SVGA_CAP_DISPLAY_TOPOLOGY   0x00080000   // Legacy multi-monitor support
#define SVGA_CAP_GMR                0x00100000
#define SVGA_CAP_TRACES             0x00200000



enum {
   /*
    * Block 1 (basic registers): The originally defined FIFO registers.
    * These exist and are valid for all versions of the FIFO protocol.
    */

   SVGA_FIFO_MIN = 0,
   SVGA_FIFO_MAX,       /* The distance from MIN to MAX must be at least 10K */
   SVGA_FIFO_NEXT_CMD,
   SVGA_FIFO_STOP,

   /*
    * Block 2 (extended registers): Mandatory registers for the extended
    * FIFO.  These exist if the SVGA caps register includes
    * SVGA_CAP_EXTENDED_FIFO; some of them are valid only if their
    * associated capability bit is enabled.
    *
    * Note that when originally defined, SVGA_CAP_EXTENDED_FIFO implied
    * support only for (FIFO registers) CAPABILITIES, FLAGS, and FENCE.
    * This means that the guest has to test individually (in most cases
    * using FIFO caps) for the presence of registers after this; the VMX
    * can define "extended FIFO" to mean whatever it wants, and currently
    * won't enable it unless there's room for that set and much more.
    */

   SVGA_FIFO_CAPABILITIES = 4,
   SVGA_FIFO_FLAGS,
   // Valid with SVGA_FIFO_CAP_FENCE:
   SVGA_FIFO_FENCE,

   /*
    * Block 3a (optional extended registers): Additional registers for the
    * extended FIFO, whose presence isn't actually implied by
    * SVGA_CAP_EXTENDED_FIFO; these exist if SVGA_FIFO_MIN is high enough to
    * leave room for them.
    *
    * These in block 3a, the VMX currently considers mandatory for the
    * extended FIFO.
    */

   // Valid if exists (i.e. if extended FIFO enabled):
   SVGA_FIFO_3D_HWVERSION,       /* See SVGA3dHardwareVersion in svga3d_reg.h */
   // Valid with SVGA_FIFO_CAP_PITCHLOCK:
   SVGA_FIFO_PITCHLOCK,

   // Valid with SVGA_FIFO_CAP_CURSOR_BYPASS_3:
   SVGA_FIFO_CURSOR_ON,          /* Cursor bypass 3 show/hide register */
   SVGA_FIFO_CURSOR_X,           /* Cursor bypass 3 x register */
   SVGA_FIFO_CURSOR_Y,           /* Cursor bypass 3 y register */
   SVGA_FIFO_CURSOR_COUNT,       /* Incremented when any of the other 3 change */
   SVGA_FIFO_CURSOR_LAST_UPDATED,/* Last time the host updated the cursor */

   // Valid with SVGA_FIFO_CAP_RESERVE:
   SVGA_FIFO_RESERVED,           /* Bytes past NEXT_CMD with real contents */

   /*
    * Valid with SVGA_FIFO_CAP_SCREEN_OBJECT:
    *
    * By default this is SVGA_ID_INVALID, to indicate that the cursor
    * coordinates are specified relative to the virtual root. If this
    * is set to a specific screen ID, cursor position is reinterpreted
    * as a signed offset relative to that screen's origin. This is the
    * only way to place the cursor on a non-rooted screen.
    */
   SVGA_FIFO_CURSOR_SCREEN_ID,

   /*
    * XXX: The gap here, up until SVGA_FIFO_3D_CAPS, can be used for new
    * registers, but this must be done carefully and with judicious use of
    * capability bits, since comparisons based on SVGA_FIFO_MIN aren't
    * enough to tell you whether the register exists: we've shipped drivers
    * and products that used SVGA_FIFO_3D_CAPS but didn't know about some of
    * the earlier ones.  The actual order of introduction was:
    * - PITCHLOCK
    * - 3D_CAPS
    * - CURSOR_* (cursor bypass 3)
    * - RESERVED
    * So, code that wants to know whether it can use any of the
    * aforementioned registers, or anything else added after PITCHLOCK and
    * before 3D_CAPS, needs to reason about something other than
    * SVGA_FIFO_MIN.
    */

   /*
    * 3D caps block space; valid with 3D hardware version >=
    * SVGA3D_HWVERSION_WS6_B1.
    */
   SVGA_FIFO_3D_CAPS      = 32,
   SVGA_FIFO_3D_CAPS_LAST = 32 + 255,

   /*
    * End of VMX's current definition of "extended-FIFO registers".
    * Registers before here are always enabled/disabled as a block; either
    * the extended FIFO is enabled and includes all preceding registers, or
    * it's disabled entirely.
    *
    * Block 3b (truly optional extended registers): Additional registers for
    * the extended FIFO, which the VMX already knows how to enable and
    * disable with correct granularity.
    *
    * Registers after here exist if and only if the guest SVGA driver
    * sets SVGA_FIFO_MIN high enough to leave room for them.
    */

   // Valid if register exists:
   SVGA_FIFO_GUEST_3D_HWVERSION, /* Guest driver's 3D version */
   SVGA_FIFO_FENCE_GOAL,         /* Matching target for SVGA_IRQFLAG_FENCE_GOAL */
   SVGA_FIFO_BUSY,               /* See "FIFO Synchronization Registers" */

   /*
    * Always keep this last.  This defines the maximum number of
    * registers we know about.  At power-on, this value is placed in
    * the SVGA_REG_MEM_REGS register, and we expect the guest driver
    * to allocate this much space in FIFO memory for registers.
    */
    SVGA_FIFO_NUM_REGS
};


#define SVGA_FIFO_EXTENDED_MANDATORY_REGS  (SVGA_FIFO_3D_CAPS_LAST + 1)





#define SVGA_FIFO_CAP_NONE                  0
#define SVGA_FIFO_CAP_FENCE             (1<<0)
#define SVGA_FIFO_CAP_ACCELFRONT        (1<<1)
#define SVGA_FIFO_CAP_PITCHLOCK         (1<<2)
#define SVGA_FIFO_CAP_VIDEO             (1<<3)
#define SVGA_FIFO_CAP_CURSOR_BYPASS_3   (1<<4)
#define SVGA_FIFO_CAP_ESCAPE            (1<<5)
#define SVGA_FIFO_CAP_RESERVE           (1<<6)
#define SVGA_FIFO_CAP_SCREEN_OBJECT     (1<<7)



#define SVGA_FIFO_FLAG_NONE                 0
#define SVGA_FIFO_FLAG_ACCELFRONT       (1<<0)
#define SVGA_FIFO_FLAG_RESERVED        (1<<31) // Internal use only


#define SVGA_FIFO_RESERVED_UNKNOWN      0xffffffff



#define SVGA_NUM_OVERLAY_UNITS 32



#define SVGA_VIDEO_FLAG_COLORKEY        0x0001



enum {
   SVGA_VIDEO_ENABLED = 0,
   SVGA_VIDEO_FLAGS,
   SVGA_VIDEO_DATA_OFFSET,
   SVGA_VIDEO_FORMAT,
   SVGA_VIDEO_COLORKEY,
   SVGA_VIDEO_SIZE,          // Deprecated
   SVGA_VIDEO_WIDTH,
   SVGA_VIDEO_HEIGHT,
   SVGA_VIDEO_SRC_X,
   SVGA_VIDEO_SRC_Y,
   SVGA_VIDEO_SRC_WIDTH,
   SVGA_VIDEO_SRC_HEIGHT,
   SVGA_VIDEO_DST_X,         // Signed int32
   SVGA_VIDEO_DST_Y,         // Signed int32
   SVGA_VIDEO_DST_WIDTH,
   SVGA_VIDEO_DST_HEIGHT,
   SVGA_VIDEO_PITCH_1,
   SVGA_VIDEO_PITCH_2,
   SVGA_VIDEO_PITCH_3,
   SVGA_VIDEO_DATA_GMRID,    // Optional, defaults to SVGA_GMR_FRAMEBUFFER
   SVGA_VIDEO_DST_SCREEN_ID, // Optional, defaults to virtual coords (SVGA_ID_INVALID)
   SVGA_VIDEO_NUM_REGS
};



typedef struct SVGAOverlayUnit {
   uint32 enabled;
   uint32 flags;
   uint32 dataOffset;
   uint32 format;
   uint32 colorKey;
   uint32 size;
   uint32 width;
   uint32 height;
   uint32 srcX;
   uint32 srcY;
   uint32 srcWidth;
   uint32 srcHeight;
   int32  dstX;
   int32  dstY;
   uint32 dstWidth;
   uint32 dstHeight;
   uint32 pitches[3];
   uint32 dataGMRId;
   uint32 dstScreenId;
} SVGAOverlayUnit;



#define SVGA_SCREEN_HAS_ROOT    (1 << 0)  // Screen is present in the virtual coord space
#define SVGA_SCREEN_IS_PRIMARY  (1 << 1)  // Guest considers this screen to be 'primary'
#define SVGA_SCREEN_FULLSCREEN_HINT (1 << 2)   // Guest is running a fullscreen app here

typedef
struct SVGAScreenObject {
   uint32 structSize;   // sizeof(SVGAScreenObject)
   uint32 id;
   uint32 flags;
   struct {
      uint32 width;
      uint32 height;
   } size;
   struct {
      int32 x;
      int32 y;
   } root;              // Only used if SVGA_SCREEN_HAS_ROOT is set.
} SVGAScreenObject;



typedef enum {
   SVGA_CMD_INVALID_CMD           = 0,
   SVGA_CMD_UPDATE                = 1,
   SVGA_CMD_RECT_COPY             = 3,
   SVGA_CMD_DEFINE_CURSOR         = 19,
   SVGA_CMD_DEFINE_ALPHA_CURSOR   = 22,
   SVGA_CMD_UPDATE_VERBOSE        = 25,
   SVGA_CMD_FRONT_ROP_FILL        = 29,
   SVGA_CMD_FENCE                 = 30,
   SVGA_CMD_ESCAPE                = 33,
   SVGA_CMD_DEFINE_SCREEN         = 34,
   SVGA_CMD_DESTROY_SCREEN        = 35,
   SVGA_CMD_DEFINE_GMRFB          = 36,
   SVGA_CMD_BLIT_GMRFB_TO_SCREEN  = 37,
   SVGA_CMD_BLIT_SCREEN_TO_GMRFB  = 38,
   SVGA_CMD_ANNOTATION_FILL       = 39,
   SVGA_CMD_ANNOTATION_COPY       = 40,
   SVGA_CMD_MAX
} SVGAFifoCmdId;

#define SVGA_CMD_MAX_ARGS           64



typedef
struct {
   uint32 x;
   uint32 y;
   uint32 width;
   uint32 height;
} SVGAFifoCmdUpdate;



typedef
struct {
   uint32 srcX;
   uint32 srcY;
   uint32 destX;
   uint32 destY;
   uint32 width;
   uint32 height;
} SVGAFifoCmdRectCopy;



typedef
struct {
   uint32 id;             // Reserved, must be zero.
   uint32 hotspotX;
   uint32 hotspotY;
   uint32 width;
   uint32 height;
   uint32 andMaskDepth;   // Value must be 1 or equal to BITS_PER_PIXEL
   uint32 xorMaskDepth;   // Value must be 1 or equal to BITS_PER_PIXEL
   /*
    * Followed by scanline data for AND mask, then XOR mask.
    * Each scanline is padded to a 32-bit boundary.
   */
} SVGAFifoCmdDefineCursor;



typedef
struct {
   uint32 id;             // Reserved, must be zero.
   uint32 hotspotX;
   uint32 hotspotY;
   uint32 width;
   uint32 height;
   /* Followed by scanline data */
} SVGAFifoCmdDefineAlphaCursor;



typedef
struct {
   uint32 x;
   uint32 y;
   uint32 width;
   uint32 height;
   uint32 reason;
} SVGAFifoCmdUpdateVerbose;



#define  SVGA_ROP_COPY                    0x03

typedef
struct {
   uint32 color;     // In the same format as the GFB
   uint32 x;
   uint32 y;
   uint32 width;
   uint32 height;
   uint32 rop;       // Must be SVGA_ROP_COPY
} SVGAFifoCmdFrontRopFill;



typedef
struct {
   uint32 fence;
} SVGAFifoCmdFence;



typedef
struct {
   uint32 nsid;
   uint32 size;
   /* followed by 'size' bytes of data */
} SVGAFifoCmdEscape;



typedef
struct {
   SVGAScreenObject screen;   // Variable-length according to version
} SVGAFifoCmdDefineScreen;



typedef
struct {
   uint32 screenId;
} SVGAFifoCmdDestroyScreen;



typedef
struct {
   SVGAGuestPtr        ptr;
   uint32              bytesPerLine;
   SVGAGMRImageFormat  format;
} SVGAFifoCmdDefineGMRFB;



typedef
struct {
   SVGASignedPoint  srcOrigin;
   SVGASignedRect   destRect;
   uint32           destScreenId;
} SVGAFifoCmdBlitGMRFBToScreen;



typedef
struct {
   SVGASignedPoint  destOrigin;
   SVGASignedRect   srcRect;
   uint32           srcScreenId;
} SVGAFifoCmdBlitScreenToGMRFB;



typedef
struct {
   SVGAColorBGRX  color;
} SVGAFifoCmdAnnotationFill;



typedef
struct {
   SVGASignedPoint  srcOrigin;
   uint32           srcScreenId;
} SVGAFifoCmdAnnotationCopy;

#endif
