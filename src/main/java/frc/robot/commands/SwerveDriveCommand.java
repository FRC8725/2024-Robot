package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.joysticks.SwerveJoystick;
import frc.robot.subsystems.SwerveSubsystem;

@SuppressWarnings("RedundantMethodOverride")
public class SwerveDriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final SwerveJoystick controller;

    public SwerveDriveCommand(SwerveSubsystem swerveSubsystem, SwerveJoystick controller) {
        this.swerveSubsystem = swerveSubsystem;
        this.controller = controller;
        this.addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        final double xSpeed = this.controller.getDesiredRobotXSpeed();
        final double ySpeed = this.controller.getDesiredRobotYSpeed();
        final double rotation = this.controller.getDesiredRobotRotation();
        boolean RightBumperDown = this.controller.getRightBumper();

        if (xSpeed != 0 || ySpeed != 0 || rotation != 0) {
            this.swerveSubsystem.setStart();
        }

        this.swerveSubsystem.drive(xSpeed, ySpeed, rotation, !RightBumperDown);
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
