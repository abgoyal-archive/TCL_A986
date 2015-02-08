
package proguard.annotation;

import java.lang.annotation.*;

@Target({ ElementType.TYPE, ElementType.FIELD, ElementType.METHOD, ElementType.CONSTRUCTOR })
@Retention(RetentionPolicy.CLASS)
@Documented
public @interface KeepName {}
