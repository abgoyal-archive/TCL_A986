
package proguard.classfile.attribute;

import proguard.classfile.Clazz;
import proguard.classfile.visitor.ClassVisitor;

public class LocalVariableInfo
{
    public int u2startPC;
    public int u2length;
    public int u2nameIndex;
    public int u2descriptorIndex;
    public int u2index;

    /**
     * An extra field pointing to the referenced Clazz object.
     * This field is typically filled out by the <code>{@link
     * proguard.classfile.util.ClassReferenceInitializer
     * ClassReferenceInitializer}</code>.
     */
    public Clazz referencedClass;


    /**
     * Creates an uninitialized LocalVariableInfo.
     */
    public LocalVariableInfo()
    {
    }


    /**
     * Creates an initialized LocalVariableInfo.
     */
    public LocalVariableInfo(int   u2startPC,
                             int   u2length,
                             int   u2nameIndex,
                             int   u2descriptorIndex,
                             int   u2index)
    {
        this.u2startPC         = u2startPC;
        this.u2length          = u2length;
        this.u2nameIndex       = u2nameIndex;
        this.u2descriptorIndex = u2descriptorIndex;
        this.u2index           = u2index;
    }


    /**
     * Lets the referenced class accept the given visitor.
     */
    public void referencedClassAccept(ClassVisitor classVisitor)
    {
        if (referencedClass != null)
        {
            referencedClass.accept(classVisitor);
        }
    }
}
