
package proguard.optimize;

import proguard.classfile.*;
import proguard.classfile.editor.ClassReferenceFixer;
import proguard.classfile.util.*;
import proguard.classfile.visitor.MemberVisitor;
import proguard.evaluation.value.Value;
import proguard.optimize.evaluation.StoringInvocationUnit;

public class MemberDescriptorSpecializer
extends      SimplifiedVisitor
implements   MemberVisitor
{
    private static final boolean DEBUG = true;


    private final MemberVisitor extraParameterMemberVisitor;


    /**
     * Creates a new MethodDescriptorShrinker.
     */
    public MemberDescriptorSpecializer()
    {
        this(null);
    }


    /**
     * Creates a new MethodDescriptorShrinker with an extra visitor.
     * @param extraParameterMemberVisitor an optional extra visitor for all
     *                                    class members whose parameters have
     *                                    been specialized.
     */
    public MemberDescriptorSpecializer(MemberVisitor extraParameterMemberVisitor)
    {
        this.extraParameterMemberVisitor = extraParameterMemberVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        Value parameterValue = StoringInvocationUnit.getFieldValue(programField);
        if (parameterValue.computationalType() == Value.TYPE_REFERENCE)
        {
            Clazz referencedClass = parameterValue.referenceValue().getReferencedClass();
            if (programField.referencedClass != referencedClass)
            {
                if (DEBUG)
                {
                    System.out.println("MemberDescriptorSpecializer: "+programClass.getName()+"."+programField.getName(programClass)+" "+programField.getDescriptor(programClass));
                    System.out.println("  "+programField.referencedClass.getName()+" -> "+referencedClass.getName());
                }

                programField.referencedClass = referencedClass;

                // Visit the field, if required.
                if (extraParameterMemberVisitor != null)
                {
                    extraParameterMemberVisitor.visitProgramField(programClass, programField);
                }
            }
        }
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        // All parameters of non-static methods are shifted by one in the local
        // variable frame.
        int firstParameterIndex =
            (programMethod.getAccessFlags() & ClassConstants.INTERNAL_ACC_STATIC) != 0 ?
                0 : 1;

        int parameterCount =
            ClassUtil.internalMethodParameterCount(programMethod.getDescriptor(programClass));

        int classIndex = 0;

        // Go over the parameters.
        for (int parameterIndex = firstParameterIndex; parameterIndex < parameterCount; parameterIndex++)
        {
            Value parameterValue = StoringInvocationUnit.getMethodParameterValue(programMethod, parameterIndex);
             if (parameterValue.computationalType() == Value.TYPE_REFERENCE)
             {
                 Clazz referencedClass = parameterValue.referenceValue().getReferencedClass();
                 if (programMethod.referencedClasses[classIndex] != referencedClass)
                 {
                     if (DEBUG)
                     {
                         System.out.println("MemberDescriptorSpecializer: "+programClass.getName()+"."+programMethod.getName(programClass)+programMethod.getDescriptor(programClass));
                         System.out.println("  "+programMethod.referencedClasses[classIndex].getName()+" -> "+referencedClass.getName());
                     }

                     programMethod.referencedClasses[classIndex] = referencedClass;

                     // Visit the method, if required.
                     if (extraParameterMemberVisitor != null)
                     {
                         extraParameterMemberVisitor.visitProgramMethod(programClass, programMethod);
                     }
                 }

                 classIndex++;
             }
        }
    }
}
