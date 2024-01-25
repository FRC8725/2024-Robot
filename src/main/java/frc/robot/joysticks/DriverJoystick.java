package frc.robot.joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.helpers.*;
import frc.robot.subsystems.SwerveSubsystem;

public class DriverJoystick extends XboxController implements IDashboardProvider {
    public static final double DEADBAND = 0.05;
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(SwerveSubsystem.TELEOP_MAX_ACCELERATION);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(SwerveSubsystem.TELEOP_MAX_ACCELERATION);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveSubsystem.TELEOP_MAX_ANGULAR_ACCELERATION);

    public DriverJoystick(int port) {
        super(port);
        this.registerDashboard();
    }

    @CoordinateSystem(CoordinationPolicy.ROBOT_COORDINATION)
    @OutputUnit(UnitTypes.METERS_PER_SECOND)
    public double getDesiredRobotXSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftY(), DEADBAND);
        return this.xSpeedLimiter.calculate(speed * SwerveSubsystem.TELEOP_MAX_ROBOT_SPEED);
    }

    @CoordinateSystem(CoordinationPolicy.ROBOT_COORDINATION)
    @OutputUnit(UnitTypes.METERS_PER_SECOND)
    public double getDesiredRobotYSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftX(), DEADBAND);
        return this.ySpeedLimiter.calculate(speed * SwerveSubsystem.TELEOP_MAX_ROBOT_SPEED);
    }

    @CoordinateSystem(CoordinationPolicy.ROBOT_COORDINATION)
    @OutputUnit(UnitTypes.RADIANS_PER_SECOND)
    public double getDesiredRobotRotation() {
        double speed = -MathUtil.applyDeadband(this.getRightX(), DEADBAND);
        return this.rotationLimiter.calculate(speed * SwerveSubsystem.TELEOP_MAX_ROBOT_ANGULAR_SPEED);
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("DesiredRobotX", this.getDesiredRobotXSpeed());
        SmartDashboard.putNumber("DesiredRobotY", this.getDesiredRobotYSpeed());
        SmartDashboard.putNumber("DesiredRobotRotation", this.getDesiredRobotRotation());
    }

    @Override
    public void putDashboardOnce() {
    }
}
