package frc.robot.commands;

import org.apache.commons.math3.util.FastMath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

@SuppressWarnings("RedundantMethodOverride")
public class AutoAimCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController steerPIDController = new PIDController(0.02, 0, 0);

    public AutoAimCommand(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.addRequirements(swerveSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
        this.steerPIDController.enableContinuousInput(-180.0, 180.0);
    }

    @Override
    public void execute() {
        this.shooterSubsystem.toggleSlopeWithDistance(this.swerveSubsystem.getSpeakerDistance());
        double angle = this.swerveSubsystem.getSpeakerAngle();
        this.swerveSubsystem.setRobotHeading(angle);
        
        // this.swerveSubsystem.drive(0.0, 0.0,
        //         this.steerPIDController.calculate(this.swerveSubsystem.getGyroAngle(), angle), true);

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
        // return FastMath.abs(this.swerveSubsystem.getGyroAngle() - this.swerveSubsystem.getSpeakerAngle()) < 2.0;
    }
}
