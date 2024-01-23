package frc.lib.helpers;

/**
 * Implementing for better convenience for {@link edu.wpi.first.wpilibj.smartdashboard.SmartDashboard} logging.
 * Follow these steps to use the interface appropriately.
 * <ol>
 *     <li>Implement this interface on any object class you'd like to log.</li>
 *     <li>Implement {@link IDashboardProvider#putDashboard()} and {@link IDashboardProvider#putDashboardOnce()}</li>
 *     <li>add {@link IDashboardProvider#registerDashboard()} in your constructors.</li>
 * </ol>
 * @see DashboardHelper
 */
public interface IDashboardProvider {
    /**
     * Put messages to {@link edu.wpi.first.wpilibj.smartdashboard.SmartDashboard} periodically
     * upon driver station receiving the code.
     * Do not put any mechanics other than logging inside, unless you know what you're doing.
     */
    void putDashboard();

    /**
     * Put messages to {@link edu.wpi.first.wpilibj.smartdashboard.SmartDashboard} for once
     * upon driver station receiving the code.
     * Do not put any mechanics other than logging inside, unless you know what you're doing.
     */
    void putDashboardOnce();

    /**
     * Register the object into the helper. Mandatory.
     */
    default void registerDashboard() {
        DashboardHelper.register(this);
    }
}
