

#ifndef ClipboardUtilitiesWin_h
#define ClipboardUtilitiesWin_h

#include "DragData.h"
#include <windows.h>

namespace WebCore {

class Document;
class KURL;
class String;

HGLOBAL createGlobalData(const String&);
HGLOBAL createGlobalData(const Vector<char>&);
HGLOBAL createGlobalData(const KURL& url, const String& title);

FORMATETC* urlWFormat();
FORMATETC* urlFormat();
FORMATETC* plainTextWFormat();
FORMATETC* plainTextFormat();
FORMATETC* filenameWFormat();
FORMATETC* filenameFormat();
FORMATETC* htmlFormat();
FORMATETC* cfHDropFormat();
FORMATETC* smartPasteFormat();

void markupToCF_HTML(const String& markup, const String& srcURL, Vector<char>& result);
String urlToMarkup(const KURL& url, const String& title);

void replaceNewlinesWithWindowsStyleNewlines(String&);
void replaceNBSPWithSpace(String&);

bool containsFilenames(const IDataObject*);
bool containsHTML(IDataObject*);

PassRefPtr<DocumentFragment> fragmentFromFilenames(Document*, const IDataObject*);
PassRefPtr<DocumentFragment> fragmentFromHTML(Document*, IDataObject*);
PassRefPtr<DocumentFragment> fragmentFromCF_HTML(Document*, const String& cf_html);

String getURL(IDataObject*, bool& success, String* title = 0);
String getPlainText(IDataObject*, bool& success);

} // namespace WebCore

#endif // ClipboardUtilitiesWin_h