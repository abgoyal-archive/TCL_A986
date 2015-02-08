
package proguard.classfile;

import proguard.classfile.visitor.*;

public class LibraryMethod extends LibraryMember implements Method
{
    /**
     * An extra field pointing to the Clazz objects referenced in the
     * descriptor string. This field is filled out by the <code>{@link
     * proguard.classfile.util.ClassReferenceInitializer ClassReferenceInitializer}</code>.
     * References to primitive types are ignored.
     */
    public Clazz[] referencedClasses;


    /**
     * Creates an uninitialized LibraryMethod.
     */
    public LibraryMethod()
    {
    }


    /**
     * Creates an initialized LibraryMethod.
     */
    public LibraryMethod(int    u2accessFlags,
                         String name,
                         String descriptor)
    {
        super(u2accessFlags, name, descriptor);
    }


    // Implementations for LibraryMember.

    public void accept(LibraryClass libraryClass, MemberVisitor memberVisitor)
    {
        memberVisitor.visitLibraryMethod(libraryClass, this);
    }


    // Implementations for Member.

    public void referencedClassesAccept(ClassVisitor classVisitor)
    {
        if (referencedClasses != null)
        {
            for (int index = 0; index < referencedClasses.length; index++)
            {
                if (referencedClasses[index] != null)
                {
                    referencedClasses[index].accept(classVisitor);
                }
            }
        }
    }
}
