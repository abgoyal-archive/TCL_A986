

#include "config.h"

#include <QApplication>

#include "Sound.h"

namespace WebCore {

void systemBeep()
{
    QApplication::beep();
}

}

// vim: ts=4 sw=4 et
