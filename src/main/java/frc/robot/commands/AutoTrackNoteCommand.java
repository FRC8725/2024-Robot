package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.MathHelper;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;
import org.apache.commons.math3.util.FastMath;

@SuppressWarnings("RedundantMethodOverride")
public class AutoTrackNoteCommand extends Command {
    private static final double TARGET_DISTANCE = 70.0;
    private static final double TARGET_HORIZONTAL = 5.0;

    private final SwerveSubsystem swerveSubsystem;
    private final VisionManager visionManager;
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveDriveConstants.AUTO_MAX_ANGULAR_ACCELERATION);
    private final PIDController steerPIDController = new PIDController(0.044, 0.0052, 0.0, 0.01);

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
        this.swerveSubsystem.situateRobot(vector.minus(new Translation2d(0.4137, 0.0)),
                vector.getAngle().getRadians(), false, true);

//        final double noteHorizontal = this.visionManager.getNoteHorizontalAngle();
//
//        double rotationSpeed = this.steerPIDController.calculate(TARGET_HORIZONTAL, noteHorizontal);
//        rotationSpeed = MathUtil.applyDeadband(rotationSpeed, 0.05);
//
//        if (this.visionManager.noNoteTarget()) {
//            rotationSpeed = 0.0;
//        }
//
//        rotationSpeed = this.rotationLimiter.calculate(rotationSpeed);
//        rotationSpeed = MathHelper.applyMax(rotationSpeed, SwerveDriveConstants.AUTO_MAX_ROBOT_ANGULAR_SPEED);
//        this.swerveSubsystem.drive(0.0, 0.0, -rotationSpeed, false);
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
