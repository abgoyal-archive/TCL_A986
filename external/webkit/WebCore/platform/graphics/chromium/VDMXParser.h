

#ifndef VDMXParser_h
#define VDMXParser_h

namespace WebCore {
    bool parseVDMX(int* ymax, int* ymin,
                   const uint8_t* vdmx, size_t vdmxLength,
                   unsigned targetPixelSize);
}  // namespace WebCore

#endif
