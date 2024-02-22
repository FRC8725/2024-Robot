package frc.robot.commands;

import org.apache.commons.math3.util.FastMath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

@SuppressWarnings("RedundantMethodOverride")
public class AutoTrackNoteCommand extends Command {
    private static final double TARGET_DISTANCE = 70.0;
    private static final double TARGET_HORIZONTAL = 7.5;

    private static final double INATKE_DOWN_DISTANCE = 120.0;
    private static final double INTAKE_DISTANCE = 90.0;

    private final SwerveSubsystem swerveSubsystem;
    private final VisionManager visionManager;
    private final IntakeSubsystem intakeSubsystem;
    
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(SwerveDriveConstants.AUTO_MAX_ACCELERATION);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveDriveConstants.AUTO_MAX_ANGULAR_ACCELERATION);
    private final PIDController drivePIDController = new PIDController(0.04, 0, 0);
    private final PIDController steerPIDController = new PIDController(0.02, 0, 0);

    public AutoTrackNoteCommand(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, VisionManager visionManager) {
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.visionManager = visionManager;
        
        this.addRequirements(swerveSubsystem, visionManager);
    }

    @Override
    public void initialize() {
    }

    // TODO: Test if can execute properly
    @Override
    public void execute() {
        final double noteDistance = this.visionManager.getNoteHorizontalDistance();
        final double noteHorizontal = this.visionManager.getNoteHorizontalAngle();

        double speed = this.drivePIDController.calculate(TARGET_DISTANCE, noteDistance);
        double rotationSpeed = MathUtil.applyDeadband(this.steerPIDController.calculate(TARGET_DISTANCE, noteHorizontal), 0.15);

        if (this.visionManager.noNoteTarget() && noteDistance < TARGET_DISTANCE) {
            speed = 0.0;
            rotationSpeed = 0.0;
        }

        speed = this.speedLimiter.calculate(speed);
        rotationSpeed = -this.rotationLimiter.calculate(rotationSpeed);

        // SmartDashboard.putNumber("Vertical_speed", verticalSpeed);
        // SmartDashboard.putNumber("Rotation_speed", rotationSpeed);

        double xSpeed = speed * FastMath.cos(Units.degreesToRadians(noteHorizontal - TARGET_HORIZONTAL));
        double ySpeed = -speed * FastMath.sin(Units.degreesToRadians(noteHorizontal - TARGET_HORIZONTAL));
        
        if (noteDistance < INATKE_DOWN_DISTANCE) this.intakeSubsystem.liftTo(IntakeSubsystem.LIFTER_MIN_SETPOINT);
        else this.intakeSubsystem.liftTo(IntakeSubsystem.LIFTER_MAX_SETPOINT);

        if (noteDistance < INTAKE_DISTANCE) this.intakeSubsystem.intake();
        else this.intakeSubsystem.stopIntake();

        this.swerveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, false);
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
