
package proguard.obfuscate;

import proguard.classfile.*;
import proguard.classfile.util.*;
import proguard.util.ListUtil;


public class MappingKeeper implements MappingProcessor
{
    private final ClassPool      classPool;
    private final WarningPrinter warningPrinter;

    // A field acting as a parameter.
    private Clazz clazz;


    /**
     * Creates a new MappingKeeper.
     * @param classPool      the class pool in which class names and class
     *                       member names have to be mapped.
     * @param warningPrinter the optional warning printer to which warnings
     *                       can be printed.
     */
    public MappingKeeper(ClassPool      classPool,
                         WarningPrinter warningPrinter)
    {
        this.classPool      = classPool;
        this.warningPrinter = warningPrinter;
    }


    // Implementations for MappingProcessor.

    public boolean processClassMapping(String className,
                                       String newClassName)
    {
        // Find the class.
        String name = ClassUtil.internalClassName(className);

        clazz = classPool.getClass(name);
        if (clazz != null)
        {
            String newName = ClassUtil.internalClassName(newClassName);

            // Print out a warning if the mapping conflicts with a name that
            // was set before.
            if (warningPrinter != null)
            {
                String currentNewName = ClassObfuscator.newClassName(clazz);
                if (currentNewName != null &&
                    !currentNewName.equals(newName))
                {
                    warningPrinter.print(name,
                                         currentNewName,
                                         "Warning: " +
                                         className +
                                         " is not being kept as '" +
                                         ClassUtil.externalClassName(currentNewName) +
                                         "', but remapped to '" +
                                         newClassName + "'");
                }
            }

            ClassObfuscator.setNewClassName(clazz, newName);

            // The class members have to be kept as well.
            return true;
        }

        return false;
    }


    public void processFieldMapping(String className,
                                    String fieldType,
                                    String fieldName,
                                    String newFieldName)
    {
        if (clazz != null)
        {
            // Find the field.
            String name       = fieldName;
            String descriptor = ClassUtil.internalType(fieldType);

            Field field = clazz.findField(name, descriptor);
            if (field != null)
            {
                // Print out a warning if the mapping conflicts with a name that
                // was set before.
                if (warningPrinter != null)
                {
                    String currentNewName = MemberObfuscator.newMemberName(field);
                    if (currentNewName != null &&
                        !currentNewName.equals(newFieldName))
                    {
                        warningPrinter.print(ClassUtil.internalClassName(className),
                                             "Warning: " +
                                             className +
                                             ": field '" + fieldType + " " + fieldName +
                                             "' is not being kept as '" + currentNewName +
                                             "', but remapped to '" + newFieldName + "'");
                    }
                }

                // Make sure the mapping name will be kept.
                MemberObfuscator.setFixedNewMemberName(field, newFieldName);
            }
        }
    }


    public void processMethodMapping(String className,
                                     int    firstLineNumber,
                                     int    lastLineNumber,
                                     String methodReturnType,
                                     String methodName,
                                     String methodArguments,
                                     String newMethodName)
    {
        if (clazz != null)
        {
            // Find the method.
            String descriptor = ClassUtil.internalMethodDescriptor(methodReturnType,
                                                                   ListUtil.commaSeparatedList(methodArguments));

            Method method = clazz.findMethod(methodName, descriptor);
            if (method != null)
            {
                // Print out a warning if the mapping conflicts with a name that
                // was set before.
                if (warningPrinter != null)
                {
                    String currentNewName = MemberObfuscator.newMemberName(method);
                    if (currentNewName != null &&
                        !currentNewName.equals(newMethodName))
                    {
                        warningPrinter.print(ClassUtil.internalClassName(className),
                                             "Warning: " +
                                             className +
                                             ": method '" + methodReturnType + " " + methodName + ClassConstants.EXTERNAL_METHOD_ARGUMENTS_OPEN + methodArguments + ClassConstants.EXTERNAL_METHOD_ARGUMENTS_CLOSE +
                                             "' is not being kept as '" + currentNewName +
                                             "', but remapped to '" + newMethodName + "'");
                    }
                }

                // Make sure the mapping name will be kept.
                MemberObfuscator.setFixedNewMemberName(method, newMethodName);
            }
        }
    }
}
