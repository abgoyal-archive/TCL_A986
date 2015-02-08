
package proguard.optimize;

import proguard.classfile.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.annotation.*;
import proguard.classfile.editor.ConstantPoolEditor;
import proguard.classfile.util.*;
import proguard.classfile.visitor.MemberVisitor;

public class DuplicateInitializerFixer
extends      SimplifiedVisitor
implements   MemberVisitor,
             AttributeVisitor
{
    private static final boolean DEBUG = false;

    private static final char[] TYPES = new char[]
    {
        ClassConstants.INTERNAL_TYPE_BYTE,
        ClassConstants.INTERNAL_TYPE_CHAR,
        ClassConstants.INTERNAL_TYPE_SHORT,
        ClassConstants.INTERNAL_TYPE_INT,
        ClassConstants.INTERNAL_TYPE_BOOLEAN
    };


    private final MemberVisitor extraFixedInitializerVisitor;


    /**
     * Creates a new DuplicateInitializerFixer.
     */
    public DuplicateInitializerFixer()
    {
        this(null);
    }


    /**
     * Creates a new DuplicateInitializerFixer with an extra visitor.
     * @param extraFixedInitializerVisitor an optional extra visitor for all
     *                                     initializers that have been fixed.
     */
    public DuplicateInitializerFixer(MemberVisitor extraFixedInitializerVisitor)
    {
        this.extraFixedInitializerVisitor = extraFixedInitializerVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        // Is it a class instance initializer?
        String name = programMethod.getName(programClass);
        if (name.equals(ClassConstants.INTERNAL_METHOD_NAME_INIT))
        {
            // Is there already another initializer with the same descriptor?
            String descriptor    = programMethod.getDescriptor(programClass);
            Method similarMethod = programClass.findMethod(name, descriptor);
            if (!programMethod.equals(similarMethod))
            {
                // Should this initializer be preserved?
                if (!KeepMarker.isKept(programMethod))
                {
                    // Fix the other initializer.
                    programMethod = (ProgramMethod)similarMethod;
                }

                int index = descriptor.indexOf(ClassConstants.INTERNAL_METHOD_ARGUMENTS_CLOSE);

                // Try to find a new, unique descriptor.
                for (int typeIndex = 0; typeIndex < TYPES.length; typeIndex++)
                {
                    String newDescriptor =
                        descriptor.substring(0, index) +
                        TYPES[typeIndex] +
                        descriptor.substring(index);

                    // Is the new initializer descriptor unique?
                    if (programClass.findMethod(name, newDescriptor) == null)
                    {
                        if (DEBUG)
                        {
                            System.out.println("DuplicateInitializerFixer:");
                            System.out.println("  ["+programClass.getName()+"]: "+name+descriptor+" -> "+newDescriptor);
                        }

                        // Update the descriptor.
                        programMethod.u2descriptorIndex =
                            new ConstantPoolEditor(programClass).addUtf8Constant(newDescriptor);

                        // Fix the local variable frame size, the method
                        // signature, and the parameter annotations, if
                        // necessary.
                        programMethod.attributesAccept(programClass,
                                                       this);

                        // Visit the initializer, if required.
                        if (extraFixedInitializerVisitor != null)
                        {
                            extraFixedInitializerVisitor.visitProgramMethod(programClass, programMethod);
                        }

                        // We're done with this constructor.
                        return;
                    }
                }

                throw new IllegalStateException("Can't find unique constructor descriptor for ["+
                                                programClass.getName()+"."+
                                                programMethod.getName(programClass)+
                                                programMethod.getDescriptor(programClass)+"]");
            }
        }
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
        // The minimum variable size is determined by the arguments.
        int maxLocals =
            ClassUtil.internalMethodParameterSize(method.getDescriptor(clazz),
                                                  method.getAccessFlags());

        if (codeAttribute.u2maxLocals < maxLocals)
        {
            codeAttribute.u2maxLocals = maxLocals;
        }
    }


    public void visitSignatureAttribute(Clazz clazz, Method method, SignatureAttribute signatureAttribute)
    {
        String descriptor      = method.getDescriptor(clazz);
        int    descriptorIndex = descriptor.indexOf(ClassConstants.INTERNAL_METHOD_ARGUMENTS_CLOSE);
        String signature       = clazz.getString(signatureAttribute.u2signatureIndex);
        int    signatureIndex  = signature.indexOf(ClassConstants.INTERNAL_METHOD_ARGUMENTS_CLOSE);

        String newSignature = signature.substring(0, signatureIndex) +
                              descriptor.charAt(descriptorIndex - 1) +
                              signature.substring(signatureIndex);

        // Update the signature.
        signatureAttribute.u2signatureIndex =
            new ConstantPoolEditor((ProgramClass)clazz).addUtf8Constant(newSignature);
    }


    public void visitAnyParameterAnnotationsAttribute(Clazz clazz, Method method, ParameterAnnotationsAttribute parameterAnnotationsAttribute)
    {
        // Update the number of parameters.
        int oldParametersCount = parameterAnnotationsAttribute.u2parametersCount++;

        if (parameterAnnotationsAttribute.u2parameterAnnotationsCount == null ||
            parameterAnnotationsAttribute.u2parameterAnnotationsCount.length < parameterAnnotationsAttribute.u2parametersCount)
        {
            int[]          annotationsCounts = new int[parameterAnnotationsAttribute.u2parametersCount];
            Annotation[][] annotations       = new Annotation[parameterAnnotationsAttribute.u2parametersCount][];

            System.arraycopy(parameterAnnotationsAttribute.u2parameterAnnotationsCount,
                             0,
                             annotationsCounts,
                             0,
                             oldParametersCount);

            System.arraycopy(parameterAnnotationsAttribute.parameterAnnotations,
                             0,
                             annotations,
                             0,
                             oldParametersCount);

            parameterAnnotationsAttribute.u2parameterAnnotationsCount = annotationsCounts;
            parameterAnnotationsAttribute.parameterAnnotations        = annotations;
        }
    }
}