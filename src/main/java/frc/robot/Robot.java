package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.helpers.DashboardHelper;
import frc.lib.helpers.TidiedUp;

@TidiedUp
@SuppressWarnings("RedundantMethodOverride")
public class Robot extends TimedRobot {
    private static final double LOOP_PERIOD = 0.01;
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public Robot() {
        super(LOOP_PERIOD);
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
        this.robotContainer.teleopPeriodic();
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

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }
}
