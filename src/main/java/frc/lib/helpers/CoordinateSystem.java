package frc.lib.helpers;

import java.lang.annotation.*;

/**
 * State the coordination system used for a method or the output of it.
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
