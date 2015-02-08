
package proguard.classfile.attribute;

import proguard.classfile.VisitorAccepter;

public class ExceptionInfo implements VisitorAccepter
{
    public int u2startPC;
    public int u2endPC;
    public int u2handlerPC;
    public int u2catchType;

    /**
     * An extra field in which visitors can store information.
     */
    public Object visitorInfo;


    /**
     * Creates an uninitialized ExceptionInfo.
     */
    public ExceptionInfo()
    {
        this(0, 0, 0, 0);
    }


    /**
     * Creates an ExceptionInfo with the given properties.
     */
    public ExceptionInfo(int u2startPC,
                         int u2endPC,
                         int u2handlerPC,
                         int u2catchType)
    {
        this.u2startPC   = u2startPC;
        this.u2endPC     = u2endPC;
        this.u2handlerPC = u2handlerPC;
        this.u2catchType = u2catchType;
    }


    /**
     * Returns whether the exception's try block contains the instruction at the
     * given offset.
     */
    public boolean isApplicable(int instructionOffset)
    {
        return instructionOffset >= u2startPC &&
               instructionOffset <  u2endPC;
    }


    /**
     * Returns whether the exception's try block overlaps with the specified
     * block of instructions.
     */
    public boolean isApplicable(int startOffset, int endOffset)
    {
        return u2startPC < endOffset &&
               u2endPC   > startOffset;
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
