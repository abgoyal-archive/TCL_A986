
package proguard.obfuscate;

import proguard.classfile.*;
import proguard.classfile.visitor.ClassVisitor;

public class NameFactoryResetter implements ClassVisitor
{
    private final NameFactory nameFactory;


    /**
     * Creates a new NameFactoryResetter.
     * @param nameFactory the name factory to be reset.
     */
    public NameFactoryResetter(NameFactory nameFactory)
    {
        this.nameFactory = nameFactory;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        nameFactory.reset();
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        nameFactory.reset();
    }
}
