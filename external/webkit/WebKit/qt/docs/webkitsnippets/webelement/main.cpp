

#include <QApplication>
#include <QUrl>
#include <qwebview.h>
#include <qwebframe.h>
#include <qwebelement.h>

static QWebFrame *frame;

static void traverse()
{
//! [Traversing with QWebElement]
    frame->setHtml("<html><body><p>First Paragraph</p><p>Second Paragraph</p></body></html>");
    QWebElement doc = frame->documentElement();
    QWebElement body = doc.firstChild();
    QWebElement firstParagraph = body.firstChild();
    QWebElement secondParagraph = firstParagraph.nextSibling();
//! [Traversing with QWebElement]
}

static void findAll()
{
//! [FindAll]
    QWebElement document = frame->documentElement();
    /* Assume the document has the following structure:

       <p class=intro>
         <span>Intro</span>
         <span>Snippets</span>
       </p>
       <p>
         <span>Content</span>
         <span>Here</span>
       </p>
    */

//! [FindAll intro]
    QWebElementCollection allSpans = document.findAll("span");
    QWebElementCollection introSpans = document.findAll("p.intro span");
//! [FindAll intro] //! [FindAll]
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QWebView *view = new QWebView(0);
    frame = view->page()->mainFrame();
    traverse();
    findAll();
    return 0;
}
