

#ifndef FormDataBuilder_h
#define FormDataBuilder_h

#include "PlatformString.h"
#include <wtf/Noncopyable.h>

namespace WebCore {

class CString;
class Document;
class TextEncoding;

class FormDataBuilder : public Noncopyable {
public:
    FormDataBuilder();
    ~FormDataBuilder();

    bool isPostMethod() const { return m_isPostMethod; }
    void setIsPostMethod(bool value) { m_isPostMethod = value; }

    bool isMultiPartForm() const { return m_isMultiPartForm; }
    void setIsMultiPartForm(bool value) { m_isMultiPartForm = value; }

    String encodingType() const { return m_encodingType; }
    void setEncodingType(const String& value) { m_encodingType = value; }

    String acceptCharset() const { return m_acceptCharset; }
    void setAcceptCharset(const String& value) { m_acceptCharset = value; }

    void parseEncodingType(const String&);
    void parseMethodType(const String&);

    TextEncoding dataEncoding(Document*) const;

    // Helper functions used by HTMLFormElement/WMLGoElement for multi-part form data
    static Vector<char> generateUniqueBoundaryString();
    static void beginMultiPartHeader(Vector<char>&, const CString& boundary, const CString& name);
    static void addBoundaryToMultiPartHeader(Vector<char>&, const CString& boundary, bool isLastBoundary = false);
    static void addFilenameToMultiPartHeader(Vector<char>&, const TextEncoding&, const String& filename);
    static void addContentTypeToMultiPartHeader(Vector<char>&, const CString& mimeType);
    static void finishMultiPartHeader(Vector<char>&);

    // Helper functions used by HTMLFormElement/WMLGoElement for non multi-part form data
    static void addKeyValuePairAsFormData(Vector<char>&, const CString& key, const CString& value);
    static void encodeStringAsFormData(Vector<char>&, const CString&);

private:
    bool m_isPostMethod;
    bool m_isMultiPartForm;

    String m_encodingType;
    String m_acceptCharset;
};

}

#endif
