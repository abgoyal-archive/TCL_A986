
package proguard.classfile.constant;

import proguard.classfile.*;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.visitor.*;

public class StringConstant extends Constant
{
    public int u2stringIndex;

    /**
     * An extra field pointing to the referenced Clazz object, if this
     * string is being used in Class.forName(), .class, or
     * Class.getDeclaredField/Method constructs.
     * This field is typically filled out by the <code>{@link
     * proguard.classfile.util.DynamicClassReferenceInitializer
     * DynamicClassReferenceInitializer}</code> or by the <code>{@link
     * proguard.classfile.util.DynamicMemberReferenceInitializer
     * DynamicMemberReferenceInitializer}</code>.
     */
    public Clazz referencedClass;

    /**
     * An extra field pointing to the referenced Member object, if this
     * string is being used in Class.getDeclaredField/Method constructs.
     * This field is typically filled out by the <code>{@link
     * proguard.classfile.util.DynamicMemberReferenceInitializer
     * DynamicMemberReferenceInitializer}</code>.
     */
    public Member referencedMember;

    /**
     * An extra field pointing to the java.lang.String Clazz object.
     * This field is typically filled out by the <code>{@link
     * proguard.classfile.util.ClassReferenceInitializer
     * ClassReferenceInitializer}</code>..
     */
    public Clazz javaLangStringClass;


    /**
     * Creates an uninitialized StringConstant.
     */
    public StringConstant()
    {
    }


    /**
     * Creates a new StringConstant with the given string index.
     * @param u2stringIndex   the index of the string in the constant pool.
     * @param referencedClass the referenced class, if any.
     * @param referenceMember the referenced class member, if any.
     */
    public StringConstant(int    u2stringIndex,
                          Clazz  referencedClass,
                          Member referenceMember)
    {
        this.u2stringIndex    = u2stringIndex;
        this.referencedClass  = referencedClass;
        this.referencedMember = referenceMember;
    }


    /**
     * Returns the string value.
     */
    public String getString(Clazz clazz)
    {
        return clazz.getString(u2stringIndex);
    }


    // Implementations for Constant.

    public int getTag()
    {
        return ClassConstants.CONSTANT_String;
    }

    public void accept(Clazz clazz, ConstantVisitor constantVisitor)
    {
        constantVisitor.visitStringConstant(clazz, this);
    }


    /**
     * Lets the referenced class accept the given visitor.
     */
    public void referencedClassAccept(ClassVisitor classVisitor)
    {
        if (referencedClass  != null &&
            referencedMember == null)
        {
            referencedClass.accept(classVisitor);
        }
    }


    /**
     * Lets the referenced member accept the given visitor.
     */
    public void referencedMemberAccept(MemberVisitor memberVisitor)
    {
        if (referencedMember != null)
        {
            referencedMember.accept(referencedClass, memberVisitor);
        }
    }
}
