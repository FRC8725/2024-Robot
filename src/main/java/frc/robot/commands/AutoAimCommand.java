package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.TrajectoryEstimator;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

@SuppressWarnings("RedundantMethodOverride")
public class AutoAimCommand extends Command {
    private static final Translation2d BLUE_SHOOTER = new Translation2d(0.0, 218.42).times(Units.inchesToMeters(1.0));
    private static final Translation2d RED_SHOOTER = new Translation2d(652.73, 218.42).times(Units.inchesToMeters(1.0));
    private final ShooterSubsystem shooterSubsystem;
    private final VisionManager visionManager;
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController drivePIDController = new PIDController(0.01, 0, 0);

    public AutoAimCommand(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, VisionManager visionManager) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.visionManager = visionManager;
        this.addRequirements(swerveSubsystem, shooterSubsystem, visionManager);
    }

    @Override
    public void initialize() {
    }

    public boolean isBlue() {
        return DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Blue).isPresent();
    }

    @Override
    public void execute() {
        Translation2d robotPos = this.swerveSubsystem.getRobotPosition().getTranslation();
        if (this.isBlue()) {
            this.shooterSubsystem.toggleSlopeWithDistance(robotPos.plus(new Translation2d(-0.33, 0.0)).getDistance(BLUE_SHOOTER));
        } else {
            this.shooterSubsystem.toggleSlopeWithDistance(robotPos.plus(new Translation2d(0.33, 0.0)).getDistance(RED_SHOOTER));
        }

//        if (this.visionManager.hasSpecTagTarget(4) || this.visionManager.hasSpecTagTarget(7)) {
//            Transform3d bestCameraToTarget = this.visionManager.getAprilTagRelative();
//
//            // Z should be the vertical distance, I want the horizontal one, so this should be
//            // sqrt(x^2 + y^2)
//            double targetDistance = bestCameraToTarget.getTranslation().getZ();
//            double targetAngle = bestCameraToTarget.getRotation().toRotation2d().getDegrees();
//
//            this.shooterSubsystem.toggleSlopeWithDistance(targetDistance);
//            // I think ...this.drivePIDController.calculate(this.swerveSubsystem.getGyroAngle(), targetAngle)... is better
//            this.swerveSubsystem.drive(0, 0, this.drivePIDController.calculate(targetAngle, 0), false);
//
//        } else {
//            this.shooterSubsystem.stopSlopeToggler();
//            this.swerveSubsystem.drive(0, 0, 0, false);
//        }
    }

    @Override
    public void end(boolean interrupted) {
        this.shooterSubsystem.stopSlopeToggler();
        this.swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
