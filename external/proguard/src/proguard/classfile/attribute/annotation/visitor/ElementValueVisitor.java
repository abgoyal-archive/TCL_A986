
package proguard.classfile.attribute.annotation.visitor;

import proguard.classfile.Clazz;
import proguard.classfile.attribute.annotation.*;

public interface ElementValueVisitor
{
    public void visitConstantElementValue(    Clazz clazz,                Annotation annotation, ConstantElementValue     constantElementValue);
    public void visitEnumConstantElementValue(Clazz clazz,                Annotation annotation, EnumConstantElementValue enumConstantElementValue);
    public void visitClassElementValue(       Clazz clazz,                Annotation annotation, ClassElementValue        classElementValue);
    public void visitAnnotationElementValue(  Clazz clazz,                Annotation annotation, AnnotationElementValue   annotationElementValue);
    public void visitArrayElementValue(       Clazz clazz,                Annotation annotation, ArrayElementValue        arrayElementValue);

//    public void visitConstantElementValue(    Clazz clazz, Field  field,  Annotation annotation, ConstantElementValue     constantElementValue);
//    public void visitEnumConstantElementValue(Clazz clazz, Field  field,  Annotation annotation, EnumConstantElementValue enumConstantElementValue);
//    public void visitClassElementValue(       Clazz clazz, Field  field,  Annotation annotation, ClassElementValue        classElementValue);
//    public void visitAnnotationElementValue(  Clazz clazz, Field  field,  Annotation annotation, AnnotationElementValue   annotationElementValue);
//    public void visitArrayElementValue(       Clazz clazz, Field  field,  Annotation annotation, ArrayElementValue        arrayElementValue);
//
//    public void visitConstantElementValue(    Clazz clazz, Method method, Annotation annotation, ConstantElementValue     constantElementValue);
//    public void visitEnumConstantElementValue(Clazz clazz, Method method, Annotation annotation, EnumConstantElementValue enumConstantElementValue);
//    public void visitClassElementValue(       Clazz clazz, Method method, Annotation annotation, ClassElementValue        classElementValue);
//    public void visitAnnotationElementValue(  Clazz clazz, Method method, Annotation annotation, AnnotationElementValue   annotationElementValue);
//    public void visitArrayElementValue(       Clazz clazz, Method method, Annotation annotation, ArrayElementValue        arrayElementValue);
}
