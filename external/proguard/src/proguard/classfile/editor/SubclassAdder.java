
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.visitor.ClassVisitor;

public class SubclassAdder
implements   ClassVisitor
{
    private final Clazz subclass;


    /**
     * Creates a new SubclassAdder that will add the given subclass.
     */
    public SubclassAdder(Clazz subclass)
    {
        this.subclass = subclass;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        programClass.addSubClass(subclass);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        libraryClass.addSubClass(subclass);
    }
}