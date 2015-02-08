
package proguard.classfile.editor;

import proguard.classfile.*;

public class ClassEditor
{
    private static final boolean DEBUG = false;

    private ProgramClass targetClass;


    /**
     * Creates a new ClassEditor that will edit elements in the given
     * target class.
     */
    public ClassEditor(ProgramClass targetClass)
    {
        this.targetClass = targetClass;
    }


    /**
     * Adds the given interface.
     */
    public void addInterface(int interfaceConstantIndex)
    {
        int   interfacesCount = targetClass.u2interfacesCount;
        int[] interfaces      = targetClass.u2interfaces;

        // Make sure there is enough space for the new interface.
        if (interfaces.length <= interfacesCount)
        {
            targetClass.u2interfaces = new int[interfacesCount+1];
            System.arraycopy(interfaces, 0,
                             targetClass.u2interfaces, 0,
                             interfacesCount);
            interfaces = targetClass.u2interfaces;
        }

        if (DEBUG)
        {
            System.out.println(targetClass.getName()+": adding interface ["+targetClass.getClassName(interfaceConstantIndex)+"]");
        }

        // Add the interface.
        interfaces[targetClass.u2interfacesCount++] = interfaceConstantIndex;
    }

    /**
     * Removes the given interface.
     */
    public void removeInterface(int interfaceConstantIndex)
    {
        int   interfacesCount = targetClass.u2interfacesCount;
        int[] interfaces      = targetClass.u2interfaces;

        int interfaceIndex = findInterfaceIndex(interfaceConstantIndex);

        // Shift the interface entries.
        System.arraycopy(interfaces, interfaceIndex+1,
                         interfaces, interfaceIndex,
                         interfacesCount - interfaceIndex - 1);

        // Clear the last entry.
        interfaces[--targetClass.u2interfacesCount] = 0;
    }


    /**
     * Finds the index of the given interface in the target class.
     */

    private int findInterfaceIndex(int interfaceConstantIndex)
    {
        int   interfacesCount = targetClass.u2interfacesCount;
        int[] interfaces      = targetClass.u2interfaces;

        for (int index = 0; index < interfacesCount; index++)
        {
            if (interfaces[index] == interfaceConstantIndex)
            {
                return index;
            }
        }

        return interfacesCount;
    }


    /**
     * Adds the given field.
     */
    public void addField(Field field)
    {
        int     fieldsCount = targetClass.u2fieldsCount;
        Field[] fields      = targetClass.fields;

        // Make sure there is enough space for the new field.
        if (fields.length <= fieldsCount)
        {
            targetClass.fields = new ProgramField[fieldsCount+1];
            System.arraycopy(fields, 0,
                             targetClass.fields, 0,
                             fieldsCount);
            fields = targetClass.fields;
        }

        if (DEBUG)
        {
            System.out.println(targetClass.getName()+": adding field ["+field.getName(targetClass)+" "+field.getDescriptor(targetClass)+"]");
        }

        // Add the field.
        fields[targetClass.u2fieldsCount++] = field;
    }


    /**
     * Removes the given field. Note that removing a field that is still being
     * referenced can cause unpredictable effects.
     */
    public void removeField(Field field)
    {
        int     fieldsCount = targetClass.u2fieldsCount;
        Field[] fields      = targetClass.fields;

        int fieldIndex = findFieldIndex(field);

        // Shift the field entries.
        System.arraycopy(fields, fieldIndex+1,
                         fields, fieldIndex,
                         fieldsCount - fieldIndex - 1);

        // Clear the last entry.
        fields[--targetClass.u2fieldsCount] = null;
    }


    /**
     * Finds the index of the given field in the target class.
     */

    private int findFieldIndex(Field field)
    {
        int     fieldsCount = targetClass.u2fieldsCount;
        Field[] fields      = targetClass.fields;

        for (int index = 0; index < fieldsCount; index++)
        {
            if (fields[index].equals(field))
            {
                return index;
            }
        }

        return fieldsCount;
    }


    /**
     * Adds the given method.
     */
    public void addMethod(Method method)
    {
        int      methodsCount = targetClass.u2methodsCount;
        Method[] methods      = targetClass.methods;

        // Make sure there is enough space for the new method.
        if (methods.length <= methodsCount)
        {
            targetClass.methods = new ProgramMethod[methodsCount+1];
            System.arraycopy(methods, 0,
                             targetClass.methods, 0,
                             methodsCount);
            methods = targetClass.methods;
        }

        if (DEBUG)
        {
            System.out.println(targetClass.getName()+": adding method ["+method.getName(targetClass)+method.getDescriptor(targetClass)+"]");
        }

        // Add the method.
        methods[targetClass.u2methodsCount++] = method;
    }


    /**
     * Removes the given method. Note that removing a method that is still being
     * referenced can cause unpredictable effects.
     */
    public void removeMethod(Method method)
    {
        int      methodsCount = targetClass.u2methodsCount;
        Method[] methods      = targetClass.methods;

        int methodIndex = findMethodIndex(method);

        // Shift the method entries.
        System.arraycopy(methods, methodIndex+1,
                         methods, methodIndex,
                         methodsCount - methodIndex - 1);

        // Clear the last entry.
        methods[--targetClass.u2methodsCount] = null;
    }


    /**
     * Finds the index of the given method in the target class.
     */

    private int findMethodIndex(Method method)
    {
        int      methodsCount = targetClass.u2methodsCount;
        Method[] methods      = targetClass.methods;

        for (int index = 0; index < methodsCount; index++)
        {
            if (methods[index].equals(method))
            {
                return index;
            }
        }

        return methodsCount;
    }
}
