
package proguard.annotation;

import java.lang.annotation.*;

@Target({ ElementType.TYPE })
@Retention(RetentionPolicy.CLASS)
@Documented
public @interface KeepGettersSetters {}
