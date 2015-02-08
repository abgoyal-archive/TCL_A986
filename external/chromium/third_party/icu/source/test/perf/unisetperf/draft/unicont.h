
class UnicodeContainable {
public:
    virtual ~UnicodeContainable() {}

    virtual UBool contains(UChar32 c) const = 0;

    virtual int32_t span(const UChar *s, int32_t length);

    virtual int32_t spanNot(const UChar *s, int32_t length);

    virtual int32_t spanUTF8(const UChar *s, int32_t length);

    virtual int32_t spanNotUTF8(const UChar *s, int32_t length);

    virtual UClassID getDynamicClassID(void) const;
};
