
package proguard.classfile;

import proguard.classfile.visitor.*;

public interface Member extends VisitorAccepter
{
    /**
     * Returns the access flags.
     */
    public int getAccessFlags();

    /**
     * Returns the class member name.
     */
    public String getName(Clazz clazz);

    /**
     * Returns the class member's descriptor.
     */
    public String getDescriptor(Clazz clazz);

    /**
     * Accepts the given class visitor.
     */
    public void accept(Clazz clazz, MemberVisitor memberVisitor);

    /**
     * Lets the Clazz objects referenced in the descriptor string
     * accept the given visitor.
     */
    public void referencedClassesAccept(ClassVisitor classVisitor);
}
