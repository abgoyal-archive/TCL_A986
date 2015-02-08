
package proguard.classfile.attribute;

import proguard.classfile.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;

public abstract class Attribute implements VisitorAccepter
{
    public int u2attributeNameIndex;
    //public int  u4attributeLength;
    //public byte info[];

    /**
     * An extra field in which visitors can store information.
     */
    public Object visitorInfo;


    /**
     * Create an uninitialized Attribute.
     */
    protected Attribute()
    {
    }


    /**
     * Create an initialized Attribute.
     */
    protected Attribute(int u2attributeNameIndex)
    {
        this.u2attributeNameIndex = u2attributeNameIndex;
    }


    /**
     * Returns the String name of the attribute.
     */
    public String getAttributeName(Clazz clazz)
    {
        return clazz.getString(u2attributeNameIndex);
    }


    // Methods to be implemented by extensions, if applicable.

    /**
     * Accepts the given visitor.
     */
    public void accept(Clazz clazz, AttributeVisitor attributeVisitor)
    {
        throw new UnsupportedOperationException("Method must be overridden in ["+this.getClass().getName()+"] if ever called");
    }

    /**
     * Accepts the given visitor in the context of the given field.
     */
    public void accept(Clazz clazz, Field field, AttributeVisitor attributeVisitor)
    {
        // Delegate the default invocation if the field is null anyway.
        if (field == null)
        {
            accept(clazz, attributeVisitor);
        }
        else
        {
            throw new UnsupportedOperationException("Method must be overridden in ["+this.getClass().getName()+"] if ever called");
        }
    }

    /**
     * Accepts the given visitor in the context of the given method.
     */
    public void accept(Clazz clazz, Method method, AttributeVisitor attributeVisitor)
    {
        // Delegate the default invocation if the method is null anyway.
        if (method == null)
        {
            accept(clazz, (Field)null, attributeVisitor);
        }
        else
        {
            throw new UnsupportedOperationException("Method must be overridden in ["+this.getClass().getName()+"] if ever called");
        }
    }

    /**
     * Accepts the given visitor in the context of the given code attribute.
     */
    public void accept(Clazz clazz, Method method, CodeAttribute codeAttribute, AttributeVisitor attributeVisitor)
    {
        // Delegate the default invocation if the code attribute is null anyway.
        if (codeAttribute == null)
        {
            accept(clazz, method, attributeVisitor);
        }
        else
        {
            throw new UnsupportedOperationException("Method must be overridden in ["+this.getClass().getName()+"] if ever called");
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
