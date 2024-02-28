package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.FieldPositions;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// TODO: Test if can execute properly
@SuppressWarnings("RedundantMethodOverride")
public class AutoAMPCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public AutoAMPCommand(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        this.addRequirements(swerveSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //.this.swerveSubsystem.situateRobot(0.0, 0.0, AMP_POSITION.getRotation().getDegrees());
        this.intakeSubsystem.liftTo(IntakeSubsystem.LIFTER_AMP_SETPOINT);

        // if (this.intakeSubsystem.isLifterAt(IntakeSubsystem.LIFTER_AMP_SETPOINT, 5.0)) {
        //     return;
        // }

        Pose2d ampPosition = Robot.isBlueAlliance() ? FieldPositions.BLUE_AMP_POSITION : FieldPositions.RED_AMP_POSITION;

        // TODO test test test test ~~~~~!!!!!!
        this.swerveSubsystem.situateRobot(ampPosition.getRotation().getRadians());
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.stopAll();
        this.swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
