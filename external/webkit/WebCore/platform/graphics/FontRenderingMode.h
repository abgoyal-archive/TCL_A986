

#ifndef FontRenderingMode_h
#define FontRenderingMode_h

namespace WebCore {

// This setting is used to provide ways of switching between multiple rendering modes that may have different
// metrics.  It is used to switch between CG and GDI text on Windows.
enum FontRenderingMode { NormalRenderingMode, AlternateRenderingMode };

} // namespace WebCore

#endif // FontRenderingMode_h
