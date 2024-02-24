package frc.robot.commands.Auto;

import org.apache.commons.math3.util.FastMath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

@SuppressWarnings("RedundantMethodOverride")
public class AutoTrackNoteCommand extends Command {
    private static final double TARGET_DISTANCE = 70.0;
    private static final double TARGET_HORIZONTAL = 5.0;

    private boolean canSteer;

    private final SwerveSubsystem swerveSubsystem;
    private final VisionManager visionManager;
    
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(SwerveDriveConstants.AUTO_MAX_ACCELERATION);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveDriveConstants.AUTO_MAX_ANGULAR_ACCELERATION);

    private final PIDController drivePIDController = new PIDController(0.03, 0, 0);
    private final PIDController steerPIDController = new PIDController(0.06, 0, 0);

    public AutoTrackNoteCommand(SwerveSubsystem swerveSubsystem, VisionManager visionManager) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionManager = visionManager;
        
        this.addRequirements(swerveSubsystem, visionManager);
    }

    @Override
    public void initialize() {
        this.canSteer = true;

    }

    @Override
    public void execute() {
        final double noteDistance = this.visionManager.getNoteHorizontalDistance();
        final double noteHorizontal = this.visionManager.getNoteHorizontalAngle();

        double speed = this.drivePIDController.calculate(TARGET_DISTANCE, noteDistance);
        
        double rotationSpeed = this.steerPIDController.calculate(TARGET_HORIZONTAL, noteHorizontal);
        rotationSpeed = MathUtil.applyDeadband(rotationSpeed, 0.05);

        if (this.visionManager.noNoteTarget() && noteDistance < TARGET_DISTANCE) {
            speed = 0.0;
            rotationSpeed = 0.0;
        }

        speed = this.speedLimiter.calculate(speed);
        rotationSpeed = -this.rotationLimiter.calculate(rotationSpeed);

        double xSpeed = speed * FastMath.cos(Units.degreesToRadians(noteHorizontal - TARGET_HORIZONTAL));
        double ySpeed = -speed * FastMath.sin(Units.degreesToRadians(noteHorizontal - TARGET_HORIZONTAL));
        
        if (rotationSpeed == 0.0 || !this.canSteer) {
            this.swerveSubsystem.drive(xSpeed, ySpeed, 0.0, false);
            this.canSteer = false;

        } else if (this.canSteer) this.swerveSubsystem.drive(0.0, 0.0, rotationSpeed, false);
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
