
package proguard.classfile;

import proguard.classfile.visitor.*;

public class LibraryField extends LibraryMember implements Field
{
    /**
     * An extra field pointing to the Clazz object referenced in the
     * descriptor string. This field is filled out by the <code>{@link
     * proguard.classfile.util.ClassReferenceInitializer ClassReferenceInitializer}</code>.
     * References to primitive types are ignored.
     */
    public Clazz referencedClass;


    /**
     * Creates an uninitialized LibraryField.
     */
    public LibraryField()
    {
    }


    /**
     * Creates an initialized LibraryField.
     */
    public LibraryField(int    u2accessFlags,
                        String name,
                        String descriptor)
    {
        super(u2accessFlags, name, descriptor);
    }


    // Implementations for LibraryMember.

    public void accept(LibraryClass libraryClass, MemberVisitor memberVisitor)
    {
        memberVisitor.visitLibraryField(libraryClass, this);
    }


    // Implementations for Member.

    public void referencedClassesAccept(ClassVisitor classVisitor)
    {
        if (referencedClass != null)
        {
            referencedClass.accept(classVisitor);
        }
    }
}
