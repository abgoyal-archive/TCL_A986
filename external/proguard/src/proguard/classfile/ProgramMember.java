
package proguard.classfile;


import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.visitor.MemberVisitor;

public abstract class ProgramMember implements Member
{
    public int         u2accessFlags;
    public int         u2nameIndex;
    public int         u2descriptorIndex;
    public int         u2attributesCount;
    public Attribute[] attributes;

    /**
     * An extra field in which visitors can store information.
     */
    public Object visitorInfo;


    /**
     * Creates an uninitialized ProgramMember.
     */
    protected ProgramMember()
    {
    }


    /**
     * Creates an initialized ProgramMember. 
     */
    protected ProgramMember(int         u2accessFlags,
                            int         u2nameIndex,
                            int         u2descriptorIndex,
                            int         u2attributesCount,
                            Attribute[] attributes)
    {
        this.u2accessFlags     = u2accessFlags;
        this.u2nameIndex       = u2nameIndex;
        this.u2descriptorIndex = u2descriptorIndex;
        this.u2attributesCount = u2attributesCount;
        this.attributes        = attributes;
    }


    /**
     * Returns the line number range of the given class member as "m:n",
     * if it can find it, or <code>null</code> otherwise.
     */
    public String getLineNumberRange(Clazz clazz)
    {
        CodeAttribute codeAttribute =
            (CodeAttribute)getAttribute(clazz, ClassConstants.ATTR_Code);
        if (codeAttribute  == null)
        {
            return null;
        }

        LineNumberTableAttribute lineNumberTableAttribute =
            (LineNumberTableAttribute)codeAttribute.getAttribute(clazz,
                                                                 ClassConstants.ATTR_LineNumberTable);
        if (lineNumberTableAttribute  == null)
        {
            return null;
        }

        return "" +
               lineNumberTableAttribute.getLineNumber(0) +
               ":" +
               lineNumberTableAttribute.getLineNumber(Integer.MAX_VALUE);
    }


    /**
     * Returns the (first) attribute with the given name.
     */
    private Attribute getAttribute(Clazz clazz, String name)
    {
        for (int index = 0; index < u2attributesCount; index++)
        {
            Attribute attribute = attributes[index];
            if (attribute.getAttributeName(clazz).equals(name))
            {
                return attribute;
            }
        }

        return null;
    }


    /**
     * Accepts the given member info visitor.
     */
    public abstract void accept(ProgramClass  programClass,
                                MemberVisitor memberVisitor);



    /**
     * Lets the given attribute info visitor visit all the attributes of
     * this member info.
     */
    public abstract void attributesAccept(ProgramClass     programClass,
                                          AttributeVisitor attributeVisitor);


    // Implementations for Member.

    public int getAccessFlags()
    {
        return u2accessFlags;
    }

    public String getName(Clazz clazz)
    {
        return clazz.getString(u2nameIndex);
    }

    public String getDescriptor(Clazz clazz)
    {
        return clazz.getString(u2descriptorIndex);
    }

    public void accept(Clazz clazz, MemberVisitor memberVisitor)
    {
        accept((ProgramClass)clazz, memberVisitor);
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
