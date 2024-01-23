package frc.lib.helpers;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;

/**
 * Mechanisms for {@link IDashboardProvider}. Implemented in {@link frc.robot.Robot}
 */
public class DashboardHelper {
    private static final ArrayList<IDashboardProvider> providers = new ArrayList<>();
    private static boolean isRegistrationValid = false;

    public static void register(IDashboardProvider provider) {
        if (isRegistrationValid) {
            providers.add(provider);
        } else {
            DriverStation.reportWarning("Found dashboard registries when DashboardHelper is invalid!", true);
        }
    }

    public static void putAllRegistriesPeriodic() {
        providers.forEach(IDashboardProvider::putDashboard);
    }

    public static void putAllRegistriesOnce() {
        providers.forEach(IDashboardProvider::putDashboardOnce);
    }

    public static void enableRegistration() {
        providers.clear();
        isRegistrationValid = true;
    }

    public static void disableRegistration() {
        isRegistrationValid = false;
    }
}
