package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

@SuppressWarnings("RedundantMethodOverride")
public class AutoTrackNoteCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final VisionManager visionManager;
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(SwerveDriveConstants.AUTO_MAX_ACCELERATION);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveDriveConstants.AUTO_MAX_ANGULAR_ACCELERATION);
    private final PIDController drivePIDController = new PIDController(0.01, 0, 0);
    private final PIDController steerPIDController = new PIDController(0.03, 0, 0);

    public AutoTrackNoteCommand(SwerveSubsystem swerveSubsystem, VisionManager visionManager) {
        this.visionManager = visionManager;
        this.swerveSubsystem = swerveSubsystem;
        this.addRequirements(swerveSubsystem, visionManager);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        final double noteDistance = this.visionManager.getNoteHorizontalDistance();
        final double noteHorizontal = this.visionManager.getNoteHorizontalAngle();

        final double targetDistance = 120.0;
        final double targetHorizontal = 17.5;

        double verticalSpeed = this.drivePIDController.calculate(targetDistance, noteDistance);
        double rotationSpeed = MathUtil.applyDeadband(this.steerPIDController.calculate(targetHorizontal, noteHorizontal), 0.15);

        if (this.visionManager.noNoteTarget() && noteDistance < targetDistance) {
            verticalSpeed = 0.0;
            rotationSpeed = 0.0;
        }

        verticalSpeed = this.speedLimiter.calculate(verticalSpeed);
        rotationSpeed = this.rotationLimiter.calculate(rotationSpeed);

        // SmartDashboard.putNumber("Vertical_speed", verticalSpeed);
        // SmartDashboard.putNumber("Rotation_speed", rotationSpeed);

        // TODO strange: vertical speed should correspond to xSpeed, not ySpeed
        this.swerveSubsystem.drive(verticalSpeed, 0.0, rotationSpeed, false);

//        if (this.visionManager.hasNoteTarget() && verticalSpeed > 0)
//            this.swerveSubsystem.move(0.0, verticalSpeed, rotationSpeed, false);
//        else this.swerveSubsystem.move(0.0, 0.0, this.visionManager.hasNoteTarget() ? rotationSpeed : 0.0, false);
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
