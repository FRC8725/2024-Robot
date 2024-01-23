package frc.lib.helpers;

import java.lang.annotation.*;

/**
 * State the appropriate output unit for the output of a method or a field.
 */
@Target({ElementType.METHOD, ElementType.FIELD})
@Retention(RetentionPolicy.SOURCE)
@Documented
public @interface OutputUnit {
    /**
     * @return the appropriate output unit.
     */
    UnitTypes value();
}
