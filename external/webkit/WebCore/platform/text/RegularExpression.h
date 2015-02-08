

#ifndef RegularExpression_h
#define RegularExpression_h

#include "PlatformString.h"

namespace WebCore {

class RegularExpression : public FastAllocBase {
public:
    RegularExpression(const String&, TextCaseSensitivity);
    ~RegularExpression();

    RegularExpression(const RegularExpression&);
    RegularExpression& operator=(const RegularExpression&);

    int match(const String&, int startFrom = 0, int* matchLength = 0) const;
    int searchRev(const String&) const;

    int matchedLength() const;

private:
    class Private;    
    RefPtr<Private> d;
};

void replace(String&, const RegularExpression&, const String&);

} // namespace WebCore

#endif // RegularExpression_h
