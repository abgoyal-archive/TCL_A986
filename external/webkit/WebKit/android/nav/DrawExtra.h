

#ifndef DrawExtra_h
#define DrawExtra_h

class SkCanvas;

namespace WebCore {
    class LayerAndroid;
}

using namespace WebCore;

namespace android {

class DrawExtra {
public:
    virtual ~DrawExtra() {}
    virtual void draw(SkCanvas* , LayerAndroid* ) = 0;
};

}

#endif
