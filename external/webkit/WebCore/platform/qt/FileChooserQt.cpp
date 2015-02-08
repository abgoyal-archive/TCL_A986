

#include "config.h"
#include "FileChooser.h"

#include "LocalizedStrings.h"
#include "Font.h"
#include <QCoreApplication>
#include <QFontMetrics>

namespace WebCore {

String FileChooser::basenameForWidth(const Font& f, int width) const
{
    if (width <= 0)
        return String();

    String string;
    if (m_filenames.isEmpty())
        string = fileButtonNoFileSelectedLabel();
    else if (m_filenames.size() == 1) {
        String fname = m_filenames[0];
        QFontMetrics fm(f.font());
        string = fm.elidedText(fname, Qt::ElideLeft, width);
    } else {
        int n = m_filenames.size();
        string = QCoreApplication::translate("QWebPage", "%n file(s)",
                                             "number of chosen file",
                                             QCoreApplication::CodecForTr, n);
    }

    return string;
}

}

// vim: ts=4 sw=4 et
