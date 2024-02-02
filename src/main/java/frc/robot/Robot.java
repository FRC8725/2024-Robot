package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.helpers.DashboardHelper;

@SuppressWarnings("RedundantMethodOverride")
public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public Robot() {
        super(0.01);
    }

    @Override
    public void robotInit() {
        DashboardHelper.enableRegistration();
        this.robotContainer = new RobotContainer();
        DashboardHelper.disableRegistration();
        DashboardHelper.putAllRegistriesOnce();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        DashboardHelper.putAllRegistriesPeriodic();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        this.autonomousCommand = this.robotContainer.getAutonomousCommand();

        if (this.autonomousCommand != null) {
            this.autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (this.autonomousCommand != null) {
            this.autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
