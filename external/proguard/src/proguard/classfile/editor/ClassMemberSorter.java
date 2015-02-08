
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.visitor.ClassVisitor;

import java.util.*;

public class ClassMemberSorter implements ClassVisitor, Comparator
{
    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Sort the fields.
        Arrays.sort(programClass.fields, 0, programClass.u2fieldsCount, this);

        // Sort the methods.
        Arrays.sort(programClass.methods, 0, programClass.u2methodsCount, this);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
    }


    // Implementations for Comparator.

    public int compare(Object object1, Object object2)
    {
        ProgramMember member1 = (ProgramMember)object1;
        ProgramMember member2 = (ProgramMember)object2;

        return member1.u2accessFlags     < member2.u2accessFlags     ? -1 :
               member1.u2accessFlags     > member2.u2accessFlags     ?  1 :
               member1.u2nameIndex       < member2.u2nameIndex       ? -1 :
               member1.u2nameIndex       > member2.u2nameIndex       ?  1 :
               member1.u2descriptorIndex < member2.u2descriptorIndex ? -1 :
               member1.u2descriptorIndex > member2.u2descriptorIndex ?  1 :
                                                                        0;
    }
}
