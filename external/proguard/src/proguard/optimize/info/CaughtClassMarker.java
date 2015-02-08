
package proguard.optimize.info;

import proguard.classfile.*;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.ClassVisitor;

public class CaughtClassMarker
implements   ClassVisitor
{
    // Implementations for ClassVisitor.

    public void visitLibraryClass(LibraryClass libraryClass) {}

    public void visitProgramClass(ProgramClass programClass)
    {
        setCaught(programClass);
    }


    // Small utility methods.

    private static void setCaught(Clazz clazz)
    {
        ClassOptimizationInfo info = ClassOptimizationInfo.getClassOptimizationInfo(clazz);
        if (info != null)
        {
            info.setCaught();
        }
    }


    public static boolean isCaught(Clazz clazz)
    {
        ClassOptimizationInfo info = ClassOptimizationInfo.getClassOptimizationInfo(clazz);
        return info == null || info.isCaught();
    }
}