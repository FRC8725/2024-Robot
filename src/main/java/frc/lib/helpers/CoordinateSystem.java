package frc.lib.helpers;

import java.lang.annotation.*;

/**
 * State the coordination system used for the output of a method.
 */
@Target({ElementType.METHOD, ElementType.FIELD})
@Retention(RetentionPolicy.SOURCE)
@Documented
public @interface CoordinateSystem {
    /**
     * @return the coordination policy.
     */
    CoordinationPolicy value();
}
