package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.TrajectoryEstimator;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

@SuppressWarnings("RedundantMethodOverride")
public class AutoAimCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final VisionManager visionManager;
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController drivePIDController = new PIDController(0.01, 0, 0);

    public AutoAimCommand(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, VisionManager visionManager) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.visionManager = visionManager;
        addRequirements(swerveSubsystem, shooterSubsystem, visionManager);
    }

    @Override
    public void initialize() {
    }

    private double getShootingSlope(double distance) {
        // return TrajectoryEstimator.getAngleOfElevation1(distance);
        return (TrajectoryEstimator.getAngleOfElevation1(distance) - 40.0) * 2.5 + 40.0;
    }

    @Override
    public void execute() {
        this.shooterSubsystem.toggleSlopeTo(this.getShootingSlope(4.0));

        // if (this.visionManager.hasSpecTagTarget(4) || this.visionManager.hasSpecTagTarget(7)) {
        //     // var bestCameraToTarget = this.visionManager.getAprilTagRelative();

        //     // double targetDistance = bestCameraToTarget.getTranslation().getZ();
        //     // double targetAngle = bestCameraToTarget.getRotation().toRotation2d().getDegrees();

        //     this.shooterSubsystem.toggleSlopeTo(this.getShootingSlope(3.0));
        //     // I think ...this.drivePIDController.calculate(this.swerveSubsystem.getGyroAngle(), targetAngle)... is better
        //     // this.swerveSubsystem.drive(0, 0, this.drivePIDController.calculate(targetAngle, 0), false);

        // } else {
        //     this.shooterSubsystem.stopSlopeToggler();
        //     this.swerveSubsystem.drive(0, 0, 0, false);
        // }
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
