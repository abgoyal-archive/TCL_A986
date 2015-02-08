
package proguard.classfile.constant;

import proguard.classfile.*;
import proguard.classfile.constant.visitor.ConstantVisitor;

public abstract class Constant implements VisitorAccepter
{
    //public int  u1tag;
    //public byte info[];

    /**
     * An extra field in which visitors can store information.
     */
    public Object visitorInfo;


    // Abstract methods to be implemented by extensions.

    /**
     * Returns the constant pool info tag that specifies the entry type.
     */
    public abstract int getTag();


    /**
     * Accepts the given visitor.
     */
    public abstract void accept(Clazz clazz, ConstantVisitor constantVisitor);


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
