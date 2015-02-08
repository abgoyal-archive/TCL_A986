
package proguard.classfile.attribute;

import proguard.classfile.Clazz;
import proguard.classfile.attribute.visitor.*;

public class InnerClassesAttribute extends Attribute
{
    public int                u2classesCount;
    public InnerClassesInfo[] classes;


    /**
     * Creates an uninitialized InnerClassesAttribute.
     */
    public InnerClassesAttribute()
    {
    }


    /**
     * Creates an initialized InnerClassesAttribute.
     */
    public InnerClassesAttribute(int                u2attributeNameIndex,
                                 int                u2classesCount,
                                 InnerClassesInfo[] classes)
    {
        super(u2attributeNameIndex);

        this.u2classesCount = u2classesCount;
        this.classes        = classes;
    }

    //

    // Implementations for Attribute.

    public void accept(Clazz clazz, AttributeVisitor attributeVisitor)
    {
        attributeVisitor.visitInnerClassesAttribute(clazz, this);
    }


    /**
     * Applies the given visitor to all inner classes.
     */
    public void innerClassEntriesAccept(Clazz clazz, InnerClassesInfoVisitor innerClassesInfoVisitor)
    {
        for (int index = 0; index < u2classesCount; index++)
        {
            // We don't need double dispatching here, since there is only one
            // type of InnerClassesInfo.
            innerClassesInfoVisitor.visitInnerClassesInfo(clazz, classes[index]);
        }
    }
}
