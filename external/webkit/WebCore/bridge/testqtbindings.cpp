
#include "config.h"

#include "Bridge.h"
#include "JSObject.h"
#include "JSValue.h"
#include "interpreter.h"
#include "qdebug.h"
#include "qobject.h"
#include "runtime_object.h"
#include "types.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>


class MyObject : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString testString READ testString WRITE setTestString)
    Q_PROPERTY(int testInt READ testInt WRITE setTestInt)

public:
    MyObject() : QObject(0), integer(0){}

    void setTestString(const QString &str) {
        qDebug() << "called setTestString" << str;
        string = str;
    }
    void setTestInt(int i) {
        qDebug() << "called setTestInt" << i;
        integer = i;
    }
    QString testString() const {
        qDebug() << "called testString" << string;
        return string;
    }
    int testInt() const {
        qDebug() << "called testInt" << integer;
        return integer;
    }
    QString string;
    int integer;

public slots:
    void foo() { qDebug() << "foo invoked"; }
};

// --------------------------------------------------------

using namespace JSC;
using namespace JSC::Bindings;

class Global : public JSObject {
public:
  virtual UString className() const { return "global"; }
};

static char code[] =
    "myInterface.foo();\n"
    "myInterface.testString = \"Hello\";\n"
    "str = myInterface.testString;\n"
    "myInterface.testInt = 10;\n"
    "i = myInterface.testInt;\n";

int main(int argc, char** argv)
{
    // expecting a filename
    bool ret = true;
    {
        JSLock lock;
        
        // create interpreter w/ global object
        Global* global = new Global();

        // create interpreter
        RefPtr<Interpreter> interp = new Interpreter(global);
        ExecState* exec = interp->globalExec();
        
        MyObject* myObject = new MyObject;
        
        global->put(exec, Identifier("myInterface"), Instance::createRuntimeObject(Instance::QtLanguage, (void*)myObject));
        
        
        if (code) {
            // run
            Completion comp(interp->evaluate("", 0, code));
            
            if (comp.complType() == Throw) {
                qDebug() << "exception thrown";
                JSValue* exVal = comp.value();
                char* msg = exVal->toString(exec).ascii();
                int lineno = -1;
                if (exVal->type() == ObjectType) {
                    JSValue* lineVal = exVal->getObject()->get(exec, Identifier("line"));
                    if (lineVal->type() == NumberType)
                        lineno = int(lineVal->toNumber(exec));
                }
                if (lineno != -1)
                    fprintf(stderr,"Exception, line %d: %s\n",lineno,msg);
                else
                    fprintf(stderr,"Exception: %s\n",msg);
                ret = false;
            }
            else if (comp.complType() == ReturnValue) {
                char* msg = comp.value()->toString(interp->globalExec()).ascii();
                fprintf(stderr,"Return value: %s\n",msg);
            }
        }
        
    } // end block, so that Interpreter and global get deleted
    
    return ret ? 0 : 1;
}

#include "testqtbindings.moc"
