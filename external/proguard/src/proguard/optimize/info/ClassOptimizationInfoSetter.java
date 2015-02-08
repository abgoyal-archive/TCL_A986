
package proguard.optimize.info;

import proguard.classfile.*;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.ClassVisitor;
import proguard.optimize.KeepMarker;

public class ClassOptimizationInfoSetter
extends      SimplifiedVisitor
implements   ClassVisitor
{
    // Implementations for MemberVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        if (!KeepMarker.isKept(programClass))
        {
            ClassOptimizationInfo.setClassOptimizationInfo(programClass);
        }
    }
}