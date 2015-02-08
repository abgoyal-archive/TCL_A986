

#ifndef qscriptconverter_p_h
#define qscriptconverter_p_h

#include <JavaScriptCore/JavaScript.h>
#include <QtCore/qstring.h>

class QScriptConverter {
public:
    static QString toString(const JSStringRef str)
    {
        return QString(reinterpret_cast<const QChar*>(JSStringGetCharactersPtr(str)), JSStringGetLength(str));
    }
    static JSStringRef toString(const QString& str)
    {
        return JSStringCreateWithUTF8CString(str.toUtf8().constData());
    }
    static JSStringRef toString(const char* str)
    {
        return JSStringCreateWithUTF8CString(str);
    }
};

#endif // qscriptconverter_p_h
