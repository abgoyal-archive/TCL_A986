
package proguard.obfuscate;

import proguard.classfile.*;
import proguard.classfile.constant.ClassConstant;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.editor.ConstantPoolEditor;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.*;

public class ClassRenamer
extends      SimplifiedVisitor
implements   ClassVisitor,
             MemberVisitor,
             ConstantVisitor
{
    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Rename this class.
        programClass.thisClassConstantAccept(this);

        // Rename the class members.
        programClass.fieldsAccept(this);
        programClass.methodsAccept(this);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        libraryClass.thisClassName = ClassObfuscator.newClassName(libraryClass);

        // Rename the class members.
        libraryClass.fieldsAccept(this);
        libraryClass.methodsAccept(this);
    }


    // Implementations for MemberVisitor.

    public void visitProgramMember(ProgramClass  programClass,
                                     ProgramMember programMember)
    {
        // Has the class member name changed?
        String name    = programMember.getName(programClass);
        String newName = MemberObfuscator.newMemberName(programMember);
        if (newName != null &&
            !newName.equals(name))
        {
            programMember.u2nameIndex =
                new ConstantPoolEditor(programClass).addUtf8Constant(newName);
        }
    }

    public void visitLibraryMember(LibraryClass  libraryClass,
                                   LibraryMember libraryMember)
    {
        String newName = MemberObfuscator.newMemberName(libraryMember);
        if (newName != null)
        {
            libraryMember.name = newName;
        }
    }


    // Implementations for ConstantVisitor.

    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        // Update the Class entry if required.
        String newName = ClassObfuscator.newClassName(clazz);
        if (newName != null)
        {
            // Refer to a new Utf8 entry.
            classConstant.u2nameIndex =
                new ConstantPoolEditor((ProgramClass)clazz).addUtf8Constant(newName);
        }
    }
}
