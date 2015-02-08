
package proguard.classfile.visitor;

import proguard.classfile.*;
import proguard.classfile.util.ClassUtil;

import java.io.PrintStream;


public class SimpleClassPrinter
implements   ClassVisitor,
             MemberVisitor
{
    private final boolean     printAccessModifiers;
    private final PrintStream ps;


    /**
     * Creates a new SimpleClassPrinter that prints to
     * <code>System.out</code>, including the access modifiers.
     */
    public SimpleClassPrinter()
    {
        this(true);
    }

    /**
     * Creates a new SimpleClassPrinter that prints to
     * <code>System.out</code>, with or without the access modifiers.
     */
    public SimpleClassPrinter(boolean printAccessModifiers)
    {
        this(printAccessModifiers, System.out);
    }

    /**
     * Creates a new SimpleClassPrinter that prints to the given
     * <code>PrintStream</code>, with or without the access modifiers.
     */
    public SimpleClassPrinter(boolean     printAccessModifiers,
                              PrintStream printStream)
    {
        this.printAccessModifiers = printAccessModifiers;
        this.ps                   = printStream;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        ps.println(ClassUtil.externalFullClassDescription(
                       printAccessModifiers ?
                           programClass.getAccessFlags() :
                           0,
                       programClass.getName()));
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        ps.println(ClassUtil.externalFullClassDescription(
                       printAccessModifiers ?
                           libraryClass.getAccessFlags() :
                           0,
                       libraryClass.getName()));
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        ps.println(ClassUtil.externalFullClassDescription(
                       printAccessModifiers ?
                           programClass.getAccessFlags() :
                           0,
                       programClass.getName()) +
                   ": " +
                   ClassUtil.externalFullFieldDescription(
                       printAccessModifiers ?
                           programField.getAccessFlags() :
                           0,
                       programField.getName(programClass),
                       programField.getDescriptor(programClass)));
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        ps.println(ClassUtil.externalFullClassDescription(
                       printAccessModifiers ?
                           programClass.getAccessFlags() :
                           0,
                       programClass.getName()) +
                   ": " +
                   ClassUtil.externalFullMethodDescription(
                       programClass.getName(),
                       printAccessModifiers ?
                           programMethod.getAccessFlags() :
                           0,
                       programMethod.getName(programClass),
                       programMethod.getDescriptor(programClass)));
    }


    public void visitLibraryField(LibraryClass libraryClass, LibraryField libraryField)
    {
        ps.println(ClassUtil.externalFullClassDescription(
                       printAccessModifiers ?
                           libraryClass.getAccessFlags() :
                           0,
                       libraryClass.getName()) +
                   ": " +
                   ClassUtil.externalFullFieldDescription(
                       printAccessModifiers ?
                           libraryField.getAccessFlags() :
                           0,
                       libraryField.getName(libraryClass),
                       libraryField.getDescriptor(libraryClass)));
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        ps.println(ClassUtil.externalFullClassDescription(
                       printAccessModifiers ?
                           libraryClass.getAccessFlags() :
                           0,
                       libraryClass.getName()) +
                   ": " +
                   ClassUtil.externalFullMethodDescription(
                       libraryClass.getName(),
                       printAccessModifiers ?
                           libraryMethod.getAccessFlags() :
                           0,
                       libraryMethod.getName(libraryClass),
                       libraryMethod.getDescriptor(libraryClass)));
    }
}
