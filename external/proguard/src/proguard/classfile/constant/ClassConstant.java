
package proguard.classfile.constant;

import proguard.classfile.*;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.visitor.ClassVisitor;

public class ClassConstant extends Constant
{
    public int u2nameIndex;

    /**
     * An extra field pointing to the referenced Clazz object.
     * This field is filled out by the <code>{@link
     * proguard.classfile.util.ClassReferenceInitializer ClassReferenceInitializer}</code>.
     */
    public Clazz referencedClass;

    /**
     * An extra field pointing to the java.lang.Class Clazz object.
     * This field is typically filled out by the <code>{@link
     * proguard.classfile.util.ClassReferenceInitializer
     * ClassReferenceInitializer}</code>..
     */
    public Clazz javaLangClassClass;


    /**
     * Creates an uninitialized ClassConstant.
     */
    public ClassConstant()
    {
    }


    /**
     * Creates a new ClassConstant with the given name index.
     * @param u2nameIndex     the index of the name in the constant pool.
     * @param referencedClass the referenced class.
     */
    public ClassConstant(int   u2nameIndex,
                         Clazz referencedClass)
    {
        this.u2nameIndex     = u2nameIndex;
        this.referencedClass = referencedClass;
    }


    /**
     * Returns the name.
     */
    public String getName(Clazz clazz)
    {
        return clazz.getString(u2nameIndex);
    }


    // Implementations for Constant.

    public int getTag()
    {
        return ClassConstants.CONSTANT_Class;
    }

    public void accept(Clazz clazz, ConstantVisitor constantVisitor)
    {
        constantVisitor.visitClassConstant(clazz, this);
    }


    /**
     * Lets the referenced class accept the given visitor.
     */
    public void referencedClassAccept(ClassVisitor classVisitor)
    {
        if (referencedClass != null)
        {
            referencedClass.accept(classVisitor);
        }
    }
}
