package frc.robot.joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.helpers.*;
import frc.robot.constants.SwerveDriveConstants;

public class DriverJoystick extends XboxController implements IDashboardProvider {
    private static final int PORT = 1;
    public static final double DEADBAND = 0.05;
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(SwerveDriveConstants.TELEOP_MAX_ACCELERATION);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(SwerveDriveConstants.TELEOP_MAX_ACCELERATION);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveDriveConstants.TELEOP_MAX_ANGULAR_ACCELERATION);

    public DriverJoystick() {
        super(PORT);
        this.registerDashboard();
    }

    public boolean isDriving() {
        return this.getDesiredRobotXSpeed() != 0 || this.getDesiredRobotYSpeed() != 0 || this.getDesiredRobotRotation() != 0;
    }

    public Trigger getZeroHeadingTrigger() {
        return new Trigger(this::getBButton);
    }

    public Trigger getModuleLockingTrigger() {
        return new Trigger(this::getXButton);
    }

    public Trigger getSpeakerAimingTrigger() {
        return new Trigger(this::getYButton);
    }

    public Trigger getNoteTrackingTrigger() {
        return new Trigger(this::getAButton);
    }

    @CoordinateSystem(CoordinationPolicy.ROBOT_COORDINATION)
    @OutputUnit(UnitTypes.METERS_PER_SECOND)
    public double getDesiredRobotXSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftY(), DEADBAND);
        return this.xSpeedLimiter.calculate(speed * SwerveDriveConstants.TELEOP_MAX_ROBOT_SPEED);
    }

    @CoordinateSystem(CoordinationPolicy.ROBOT_COORDINATION)
    @OutputUnit(UnitTypes.METERS_PER_SECOND)
    public double getDesiredRobotYSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftX(), DEADBAND);
        return this.ySpeedLimiter.calculate(speed * SwerveDriveConstants.TELEOP_MAX_ROBOT_SPEED);
    }

    @CoordinateSystem(CoordinationPolicy.ROBOT_COORDINATION)
    @OutputUnit(UnitTypes.RADIANS_PER_SECOND)
    public double getDesiredRobotRotation() {
        double speed = -MathUtil.applyDeadband(this.getRightX(), DEADBAND);
        return this.rotationLimiter.calculate(speed * SwerveDriveConstants.TELEOP_MAX_ROBOT_ANGULAR_SPEED);
    }

    public boolean isFieldOriented() {
        return !this.getRightBumper();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("DesiredRobotX", this.getDesiredRobotXSpeed());
        SmartDashboard.putNumber("DesiredRobotY", this.getDesiredRobotYSpeed());
        SmartDashboard.putNumber("DesiredRobotRotation", this.getDesiredRobotRotation());
        SmartDashboard.putBoolean("isFieldOriented", this.isFieldOriented());
    }

    @Override
    public void putDashboardOnce() {
    }
}
