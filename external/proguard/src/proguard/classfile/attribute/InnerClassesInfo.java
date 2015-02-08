
package proguard.classfile.attribute;

import proguard.classfile.*;
import proguard.classfile.constant.visitor.ConstantVisitor;

public class InnerClassesInfo implements VisitorAccepter
{
    public int u2innerClassIndex;
    public int u2outerClassIndex;
    public int u2innerNameIndex;
    public int u2innerClassAccessFlags;

    /**
     * An extra field in which visitors can store information.
     */
    public Object visitorInfo;


    /**
     * Returns the inner class index.
     */
    protected int getInnerClassIndex()
    {
        return u2innerClassIndex;
    }

    /**
     * Returns the name index.
     */
    protected int getInnerNameIndex()
    {
        return u2innerNameIndex;
    }

    /**
     * Sets the name index.
     */
    protected void setInnerNameIndex(int index)
    {
        u2innerNameIndex = index;
    }


    /**
     * Applies the given constant pool visitor to the class constant of the
     * inner class, if any.
     */
    public void innerClassConstantAccept(Clazz clazz, ConstantVisitor constantVisitor)
    {
        if (u2innerClassIndex != 0)
        {
            clazz.constantPoolEntryAccept(u2innerClassIndex,
                                                 constantVisitor);
        }
    }


    /**
     * Applies the given constant pool visitor to the class constant of the
     * outer class, if any.
     */
    public void outerClassConstantAccept(Clazz clazz, ConstantVisitor constantVisitor)
    {
        if (u2outerClassIndex != 0)
        {
            clazz.constantPoolEntryAccept(u2outerClassIndex,
                                                 constantVisitor);
        }
    }


    /**
     * Applies the given constant pool visitor to the Utf8 constant of the
     * inner name, if any.
     */
    public void innerNameConstantAccept(Clazz clazz, ConstantVisitor constantVisitor)
    {
        if (u2innerNameIndex != 0)
        {
            clazz.constantPoolEntryAccept(u2innerNameIndex,
                                                 constantVisitor);
        }
    }


    // Implementations for VisitorAccepter.

    public Object getVisitorInfo()
    {
        return visitorInfo;
    }

    public void setVisitorInfo(Object visitorInfo)
    {
        this.visitorInfo = visitorInfo;
    }
}
