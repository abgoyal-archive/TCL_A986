
package proguard.optimize.evaluation;

import proguard.classfile.*;
import proguard.classfile.constant.RefConstant;
import proguard.evaluation.BasicInvocationUnit;
import proguard.evaluation.value.*;

public class LoadingInvocationUnit
extends      BasicInvocationUnit
{
    private boolean loadFieldValues;
    private boolean loadMethodParameterValues;
    private boolean loadMethodReturnValues;


    /**
     * Creates a new LoadingInvocationUnit with the given value factory.
     */
    public LoadingInvocationUnit(ValueFactory valueFactory)
    {
        this(valueFactory, false, false, false);
    }


    /**
     * Creates a new LoadingInvocationUnit with the given value factory, for
     * loading the specified values.
     */
    public LoadingInvocationUnit(ValueFactory valueFactory,
                                 boolean      loadFieldValues,
                                 boolean      loadMethodParameterValues,
                                 boolean      loadMethodReturnValues)
    {
        super(valueFactory);

        this.loadFieldValues           = loadFieldValues;
        this.loadMethodParameterValues = loadMethodParameterValues;
        this.loadMethodReturnValues    = loadMethodReturnValues;
    }


    // Implementations for BasicInvocationUnit.

    protected Value getFieldClassValue(Clazz       clazz,
                                       RefConstant refConstant,
                                       String      type)
    {
        if (loadFieldValues)
        {
            // Do we know this field?
            Member referencedMember = refConstant.referencedMember;
            if (referencedMember != null)
            {
                // Retrieve the stored field class value.
                ReferenceValue value = StoringInvocationUnit.getFieldClassValue((Field)referencedMember);
                if (value != null &&
                    value.isParticular())
                {
                    return value;
//                // Make sure the value is refreshed.
//                return refresh(value);
                }
            }
        }

        return super.getFieldClassValue(clazz, refConstant, type);
    }


    protected Value getFieldValue(Clazz       clazz,
                                  RefConstant refConstant,
                                  String      type)
    {
        if (loadFieldValues)
        {
            // Do we know this field?
            Member referencedMember = refConstant.referencedMember;
            if (referencedMember != null)
            {
                // Retrieve the stored field value.
                Value value = StoringInvocationUnit.getFieldValue((Field)referencedMember);
                if (value != null &&
                    value.isParticular())
                {
                    return value;
//                // Make sure the value is refreshed.
//                return refresh(value);
                }
            }
        }

        return super.getFieldValue(clazz, refConstant, type);
    }


    protected Value getMethodParameterValue(Clazz  clazz,
                                            Method method,
                                            int    parameterIndex,
                                            String type,
                                            Clazz  referencedClass)
    {
        if (loadMethodParameterValues)
        {
            // Retrieve the stored method parameter value.
            Value value = StoringInvocationUnit.getMethodParameterValue(method, parameterIndex);
            if (value != null &&
                value.isParticular())
            {
                return value;
//            // Make sure the value is refreshed.
//            return refresh(value);
            }
        }

        return super.getMethodParameterValue(clazz,
                                             method,
                                             parameterIndex,
                                             type,
                                             referencedClass);
    }


    protected Value getMethodReturnValue(Clazz       clazz,
                                         RefConstant refConstant,
                                         String      type)
    {
        if (loadMethodReturnValues)
        {
            // Do we know this method?
            Member referencedMember = refConstant.referencedMember;
            if (referencedMember != null)
            {
                // Retrieve the stored method return value.
                Value value = StoringInvocationUnit.getMethodReturnValue((Method)referencedMember);
                if (value != null &&
                    value.isParticular())
                {
                    return value;
//                // Make sure the value is refreshed.
//                return refresh(value);
                }
            }
        }

        return super.getMethodReturnValue(clazz,
                                          refConstant,
                                          type);
    }
//
//
//    // Small utility methods.
//
//    private Value refresh(Value value)
//    {
//        if (value.isParticular())
//        {
//            return value;
//        }
//
//        switch (value.computationalType())
//        {
//            case Value.TYPE_INTEGER: return valueFactory.createIntegerValue();
//            case Value.TYPE_LONG:    return valueFactory.createLongValue();
//            case Value.TYPE_FLOAT:   return valueFactory.createFloatValue();
//            case Value.TYPE_DOUBLE:  return valueFactory.createDoubleValue();
//            default:
//            {
//                ReferenceValue referenceValue = value.referenceValue();
//
//                return valueFactory.createReferenceValue(referenceValue.getType(),
//                                                         referenceValue.getReferencedClass(),
//                                                         referenceValue.isNull() != Value.NEVER);
//            }
//        }
//    }
}
