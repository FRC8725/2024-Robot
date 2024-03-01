package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

@SuppressWarnings("RedundantMethodOverride")
public class AutoTrackNoteCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final VisionManager visionManager;

    public AutoTrackNoteCommand(SwerveSubsystem swerveSubsystem, VisionManager visionManager) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionManager = visionManager;

        this.addRequirements(swerveSubsystem, visionManager);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (this.visionManager.noNoteTarget() || this.visionManager.getNotePositionVector() == null || this.visionManager.getNoteGroundDistance() < 0.1) {
            this.swerveSubsystem.drive(0.0, 0.0, 0.0, false);
            return;
        }

        Translation2d vector = this.visionManager.getNotePositionVector();
        this.swerveSubsystem.situateRobot(vector, vector.getAngle().getRadians(), false, true);
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
