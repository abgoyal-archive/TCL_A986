
package proguard.classfile.attribute.preverification;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.preverification.visitor.StackMapFrameVisitor;
import proguard.classfile.attribute.visitor.AttributeVisitor;

public class StackMapTableAttribute extends Attribute
{
    public int             u2stackMapFramesCount;
    public StackMapFrame[] stackMapFrames;


    /**
     * Creates an uninitialized StackMapTableAttribute.
     */
    public StackMapTableAttribute()
    {
    }


    /**
     * Creates a StackMapTableAttribute with the given stack map frames.
     */
    public StackMapTableAttribute(StackMapFrame[] stackMapFrames)
    {
        this(stackMapFrames.length, stackMapFrames);
    }


    /**
     * Creates a StackMapTableAttribute with the given stack map frames.
     */
    public StackMapTableAttribute(int             stackMapFramesCount,
                                  StackMapFrame[] stackMapFrames)
    {
        this.u2stackMapFramesCount = stackMapFramesCount;
        this.stackMapFrames        = stackMapFrames;
    }


    // Implementations for Attribute.

    public void accept(Clazz clazz, Method method, CodeAttribute codeAttribute, AttributeVisitor attributeVisitor)
    {
        attributeVisitor.visitStackMapTableAttribute(clazz, method, codeAttribute, this);
    }


    /**
     * Applies the given stack map frame visitor to all stack map frames.
     */
    public void stackMapFramesAccept(Clazz clazz, Method method, CodeAttribute codeAttribute, StackMapFrameVisitor stackMapFrameVisitor)
    {
        int offset = 0;

        for (int index = 0; index < u2stackMapFramesCount; index++)
        {
            StackMapFrame stackMapFrame = stackMapFrames[index];

            // Note that the byte code offset is computed differently for the
            // first stack map frame.
            offset += stackMapFrame.getOffsetDelta() + (index == 0 ? 0 : 1);

            stackMapFrame.accept(clazz, method, codeAttribute, offset, stackMapFrameVisitor);
        }
    }
}
