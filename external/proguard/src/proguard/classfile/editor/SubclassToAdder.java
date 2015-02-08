
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.visitor.ClassVisitor;

public class SubclassToAdder
implements   ClassVisitor
{
    private final Clazz targetClass;


    /**
     * Creates a new SubclassAdder that will add subclasses to the given
     * target class.
     */
    public SubclassToAdder(Clazz targetClass)
    {
        this.targetClass = targetClass;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        targetClass.addSubClass(programClass);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        targetClass.addSubClass(libraryClass);
    }
}