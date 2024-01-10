package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.subsystems.SwerveSubsystem;

@SuppressWarnings("RedundantMethodOverride")
public class DriveJoystickCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController controller;

    public DriveJoystickCmd(SwerveSubsystem swerveSubsystem, XboxController controller) {
        this.swerveSubsystem = swerveSubsystem;
        this.controller = controller;
        this.addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Get input
        double xSpeed = this.controller.getLeftX();
        double ySpeed = this.controller.getLeftY();
        double rotation = this.controller.getRightX();
        boolean RightBumperDown = this.controller.getRightBumper();

        // Apply deadband
        xSpeed = MathUtil.applyDeadband(xSpeed, DriverJoystick.DEADBAND);
        ySpeed = MathUtil.applyDeadband(ySpeed, DriverJoystick.DEADBAND);
        rotation = MathUtil.applyDeadband(rotation, DriverJoystick.DEADBAND);

        // Drive
        this.swerveSubsystem.move(xSpeed, -ySpeed, rotation, !RightBumperDown);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
