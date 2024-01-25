package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

public class AutoTrackNoteCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController autoTrackPidController = new PIDController(0.01, 0, 0);
    private final PIDController autoTrackThetaPidController = new PIDController(0.03, 0, 0);
    private final VisionManager visionManager;

    public AutoTrackNoteCommand(SwerveSubsystem swerveSubsystem, VisionManager visionManager) {
        this.visionManager = visionManager;
        this.swerveSubsystem = swerveSubsystem;
        this.addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final double note_distance = this.visionManager.getNoteDistance();
        final double note_horizontal = this.visionManager.getNoteHorizontal();

        final double target_distance = 120.0;
        final double target_horizontal = 9.2;

        final double vertical_speed = autoTrackPidController.calculate(target_distance, note_distance);
        final double rotation_speed = MathUtil.applyDeadband(autoTrackThetaPidController.calculate(target_horizontal, note_horizontal), 0.15);

        // SmartDashboard.putNumber("Vertical_speed", vertical_speed);
        // SmartDashboard.putNumber("Rotation_speed", rotation_speed);

        if (this.visionManager.hasNoteTarget() && vertical_speed > 0)
            this.swerveSubsystem.move(0.0, vertical_speed, rotation_speed, false);
        else this.swerveSubsystem.move(0.0, 0.0, this.visionManager.hasNoteTarget() ? rotation_speed : 0.0, false);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.swerveSubsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}