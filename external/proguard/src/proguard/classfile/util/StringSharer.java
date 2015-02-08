
package proguard.classfile.util;

import proguard.classfile.*;
import proguard.classfile.attribute.Attribute;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.constant.*;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.visitor.ClassVisitor;

public class StringSharer
extends      SimplifiedVisitor
implements   ClassVisitor,
             ConstantVisitor,
             AttributeVisitor
{
    // A fields acting as an argument for the visitor methods.
    private String name;
    private String type;


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Replace name strings in the constant pool by shared strings.
        programClass.constantPoolEntriesAccept(this);

        // Replace attribute name strings in the constant pool by internalized
        // strings.
        programClass.attributesAccept(this);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        // Replace the super class name string by the shared name string.
        Clazz superClass = libraryClass.superClass;
        if (superClass != null)
        {
            libraryClass.superClassName = superClass.getName();
        }
    }


    // Implementations for ConstantVisitor.


    public void visitAnyConstant(Clazz clazz, Constant constant) {}


    public void visitAnyStringConstant(Clazz clazz, StringConstant stringConstant)
    {
        Member referencedMember = stringConstant.referencedMember;
        if (referencedMember != null)
        {
            Clazz referencedClass = stringConstant.referencedClass;

            // Put the actual class member's name in the class pool.
            name = referencedMember.getName(referencedClass);
            clazz.constantPoolEntryAccept(stringConstant.u2stringIndex, this);
        }
    }


    public void visitAnyRefConstant(Clazz clazz, RefConstant refConstant)
    {
        Member referencedMember = refConstant.referencedMember;
        if (referencedMember != null)
        {
            Clazz referencedClass = refConstant.referencedClass;

            // Put the actual class member's name and type strings in the class
            // pool.
            name = referencedMember.getName(referencedClass);
            type = referencedMember.getDescriptor(referencedClass);
            clazz.constantPoolEntryAccept(refConstant.u2nameAndTypeIndex, this);
        }
    }


    public void visitNameAndTypeConstant(Clazz clazz, NameAndTypeConstant nameAndTypeConstant)
    {
        if (name != null)
        {
            // Put the actual class member's name and type strings in the class
            // pool.
            clazz.constantPoolEntryAccept(nameAndTypeConstant.u2nameIndex, this);
            name = type;
            clazz.constantPoolEntryAccept(nameAndTypeConstant.u2descriptorIndex, this);
        }
    }


    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        Clazz referencedClass = classConstant.referencedClass;
        if (referencedClass != null)
        {
            // Put the actual class's name string in the class pool.
            name = referencedClass.getName();
            clazz.constantPoolEntryAccept(classConstant.u2nameIndex, this);
        }
    }


    public void visitUtf8Constant(Clazz clazz, Utf8Constant utf8Constant)
    {
        // Do we have a new string to put into this constant?
        if (name != null)
        {
            // Replace the string, if it's actually the same.
            if (name.equals(utf8Constant.getString()))
            {
                utf8Constant.setString(name);
            }

            name = null;
        }
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute)
    {
        // Put the internalized attribute's name string in the class pool.
        name = attribute.getAttributeName(clazz).intern();
        clazz.constantPoolEntryAccept(attribute.u2attributeNameIndex, this);
    }
}
