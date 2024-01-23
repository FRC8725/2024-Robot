package frc.robot.joysticks;

import edu.wpi.first.wpilibj.XboxController;
import frc.lib.helpers.CoordinateSystem;
import frc.lib.helpers.CoordinationPolicy;
import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.UnitTypes;
import frc.robot.commands.DriveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystick extends XboxController {
    public static final double DEADBAND = 0.05;

    public SwerveJoystick(int port) {
        super(port);
    }

    public void setDefaultCommand(SwerveSubsystem subsystem) {
        DriveJoystickCmd cmd = new DriveJoystickCmd(subsystem, this);
        subsystem.setDefaultCommand(cmd);
    }

    @CoordinateSystem(CoordinationPolicy.ROBOT_COORDINATION)
    @OutputUnit(UnitTypes.METERS_PER_SECOND)
    public void getDesiredRobotXSpeed() {
        double rawValue = -this.getLeftY();
    }

    @CoordinateSystem(CoordinationPolicy.ROBOT_COORDINATION)
    @OutputUnit(UnitTypes.METERS_PER_SECOND)
    public void getDesiredRobotYSpeed() {

    }

    @CoordinateSystem(CoordinationPolicy.ROBOT_COORDINATION)
    @OutputUnit(UnitTypes.RADIANS_PER_SECOND)
    public void getDesiredRobotRotation() {

    }
}
