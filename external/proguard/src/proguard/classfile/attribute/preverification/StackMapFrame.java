
package proguard.classfile.attribute.preverification;

import proguard.classfile.*;
import proguard.classfile.attribute.CodeAttribute;
import proguard.classfile.attribute.preverification.visitor.StackMapFrameVisitor;

public abstract class StackMapFrame implements VisitorAccepter
{
    public static final int SAME_ZERO_FRAME          =   0;
    public static final int SAME_ONE_FRAME           =  64;
    public static final int SAME_ONE_FRAME_EXTENDED  = 247;
    public static final int LESS_ZERO_FRAME          = 248;
    public static final int SAME_ZERO_FRAME_EXTENDED = 251;
    public static final int MORE_ZERO_FRAME          = 252;
    public static final int FULL_FRAME               = 255;


    public int u2offsetDelta;

    /**
     * An extra field in which visitors can store information.
     */
    public Object visitorInfo;



    /**
     * Returns the bytecode offset delta relative to the previous stack map
     * frame.
     */
    public int getOffsetDelta()
    {
        return u2offsetDelta;
    }


    // Abstract methods to be implemented by extensions.

    /**
     * Returns the stack map frame tag that specifies the entry type.
     */
    public abstract int getTag();


    /**
     * Accepts the given visitor.
     */
    public abstract void accept(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, StackMapFrameVisitor stackMapFrameVisitor);


    // Implementations for VisitorAccepter.

    public Object getVisitorInfo()
    {
        return visitorInfo;
    }

    public void setVisitorInfo(Object visitorInfo)
    {
        this.visitorInfo = visitorInfo;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        if (object == null ||
            this.getClass() != object.getClass())
        {
            return false;
        }

        StackMapFrame other = (StackMapFrame)object;

        return this.u2offsetDelta == other.u2offsetDelta;
    }


    public int hashCode()
    {
        return getClass().hashCode() ^
               u2offsetDelta;
    }


    public String toString()
    {
        return "[" + u2offsetDelta + "] ";
    }
}
