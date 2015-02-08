
package proguard.optimize;

import proguard.classfile.*;
import proguard.classfile.visitor.MemberVisitor;
import proguard.optimize.info.*;

public class OptimizationInfoMemberFilter
implements   MemberVisitor
{
    private final MemberVisitor memberVisitor;


    /**
     * Creates a new OptimizationInfoMemberFilter.
     * @param memberVisitor the <code>MemberVisitor</code> to which visits will
     *                      be delegated.
     */
    public OptimizationInfoMemberFilter(MemberVisitor memberVisitor)
    {
        this.memberVisitor = memberVisitor;
    }


    // Implementations for MemberVisitor.


    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        // Does the field have optimization info?
        if (FieldOptimizationInfo.getFieldOptimizationInfo(programField) != null)
        {
            memberVisitor.visitProgramField(programClass, programField);
        }
    }


    public void visitLibraryField(LibraryClass libraryClass, LibraryField libraryField)
    {
        // Does the field have optimization info?
        if (FieldOptimizationInfo.getFieldOptimizationInfo(libraryField) != null)
        {
            memberVisitor.visitLibraryField(libraryClass, libraryField);
        }
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        // Does the method have optimization info?
        if (MethodOptimizationInfo.getMethodOptimizationInfo(programMethod) != null)
        {
            memberVisitor.visitProgramMethod(programClass, programMethod);
        }
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        // Does the method have optimization info?
        if (MethodOptimizationInfo.getMethodOptimizationInfo(libraryMethod) != null)
        {
            memberVisitor.visitLibraryMethod(libraryClass, libraryMethod);
        }
    }
}
