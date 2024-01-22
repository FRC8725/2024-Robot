package frc.lib.helpers;

public interface IDashboardProvider {
    void putDashboard();

    void putDashboardOnce();

    default void registerDashboard() {
        DashboardHelper.register(this);
        this.putDashboard();
        this.putDashboardOnce();
    }
}
