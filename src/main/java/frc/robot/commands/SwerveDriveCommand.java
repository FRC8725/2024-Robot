package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.helpers.TidiedUp;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.subsystems.SwerveSubsystem;

@TidiedUp
@SuppressWarnings("RedundantMethodOverride")
public class SwerveDriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final DriverJoystick joystick;
    private boolean isSteeringLocked;

    public SwerveDriveCommand(SwerveSubsystem swerveSubsystem, DriverJoystick joystick) {
        this.swerveSubsystem = swerveSubsystem;
        this.joystick = joystick;
        this.addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.isSteeringLocked = true;
    }

    @Override
    public void execute() {
        // TODO test if this still works upon re-entering the teleop mode
        if (this.joystick.isDriving()) {
            this.isSteeringLocked = false;
        } else if (this.isSteeringLocked) {
            return;
        }

        final double xSpeed = this.joystick.getDesiredRobotXSpeed();
        final double ySpeed = this.joystick.getDesiredRobotYSpeed();
        final double rotation = this.joystick.getDesiredRobotRotation();
        final boolean fieldOriented = this.joystick.isFieldOriented();
        this.swerveSubsystem.drive(xSpeed, ySpeed, rotation, fieldOriented);
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
