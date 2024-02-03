// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

public class AutoAimCommand extends Command {
  /** Creates a new AutoAimCommand. */
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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private double getShootingslope(double distance) {
    return 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (visionManager.hasSpecTagTarget(4) || visionManager.hasSpecTagTarget(7)) {
      var bestCameraToTarget = visionManager.getAprilTagRelative();
      
      double targetDistance = bestCameraToTarget.getTranslation().getZ();
      double targetAngle = bestCameraToTarget.getRotation().toRotation2d().getDegrees();

      shooterSubsystem.toggleSlopeTo(getShootingslope(targetDistance));
      swerveSubsystem.drive(0, 0, drivePIDController.calculate(targetAngle, 0), false);
      
    } else {
      shooterSubsystem.stopSlopeToggler();
      swerveSubsystem.drive(0, 0, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopSlopeToggler();
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
