
package proguard.classfile.attribute.annotation;

import proguard.classfile.*;
import proguard.classfile.attribute.annotation.visitor.ElementValueVisitor;
import proguard.classfile.visitor.MemberVisitor;

public abstract class ElementValue implements VisitorAccepter
{
    /**
     * An extra field for the optional element name. It is used in element value
     * pairs of annotations. Otherwise, it is 0.
     */
    public int u2elementNameIndex;

    /**
     * An extra field pointing to the referenced <code>Clazz</code>
     * object, if applicable. This field is typically filled out by the
     * <code>{@link proguard.classfile.util.ClassReferenceInitializer}</code>.
     */
    public Clazz referencedClass;

    /**
     * An extra field pointing to the referenced <code>Method</code>
     * object, if applicable. This field is typically filled out by the
     * <code>{@link proguard.classfile.util.ClassReferenceInitializer}</code>.
     */
    public Method referencedMethod;

    /**
     * An extra field in which visitors can store information.
     */
    public Object visitorInfo;


    /**
     * Creates an uninitialized ElementValue.
     */
    protected ElementValue()
    {
    }


    /**
     * Creates an initialized ElementValue.
     */
    protected ElementValue(int u2elementNameIndex)
    {
        this.u2elementNameIndex = u2elementNameIndex;
    }


    /**
     * Returns the element name.
     */
    public String getMethodName(Clazz clazz)
    {
        return clazz.getString(u2elementNameIndex);
    }


    // Abstract methods to be implemented by extensions.

    /**
     * Returns the tag of this element value.
     */
    public abstract int getTag();


    /**
     * Accepts the given visitor.
     */
    public abstract void accept(Clazz clazz, Annotation annotation, ElementValueVisitor elementValueVisitor);



    /**
     * Applies the given visitor to the referenced method.
     */
    public void referencedMethodAccept(MemberVisitor memberVisitor)
    {
        if (referencedMethod != null)
        {
            referencedMethod.accept(referencedClass, memberVisitor);
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
