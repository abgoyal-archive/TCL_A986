

#ifndef SurfaceCallback_H
#define SurfaceCallback_H

namespace android {

    class SurfaceCallback {
    public:
        virtual ~SurfaceCallback() {}
        virtual void surfaceCreated() = 0;
        virtual void surfaceChanged(int format, int width, int height) = 0;
        virtual void surfaceDestroyed() = 0;
    };

} // namespace android

#endif
