// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

public class AutoTrackCmd extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final PIDController autoTrackPidController = new PIDController(Constants.VisionConstants.kAutoTrackYP, 0, 0);
  private final PIDController autoTrackRotationPidController = new PIDController(Constants.VisionConstants.kAutoTrackThetaP, 0, 0);
  private final VisionManager visionManager;

  /** Creates a new AutoTrackCmd. */
  public AutoTrackCmd(SwerveSubsystem swerveSubsystem, VisionManager visionManager) {
    this.visionManager = visionManager;
    this.swerveSubsystem = swerveSubsystem;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var relativePos = visionManager.getAprilTagRelative();
    SmartDashboard.putString("Transfrom3d", relativePos.toString());

    // double apriltag_id = VisionManager.getApriltagId();
    // double distanceToGoalInches = VisionManager.getdistanceToGoalInches();
    // double distanceToGoalMeters = Units.inchesToMeters(distanceToGoalInches);
    // double distanceToGoalHorizontalInches = VisionManager.getDistanceToGoalHorizontalInches(distanceToGoalInches);
    
    // double LimitDistance = 3.78;
    // double verticalSpeed = autoTrackPidController.calculate(LimitDistance, distanceToGoalMeters);
    // double rotationSpeed = MathUtil.applyDeadband(autoTrackRotationPidController.calculate(0, distanceToGoalHorizontalInches), 0.1);

    // // double verticalSpeed = Constants.VisionConstants.kAutotrackMaxSpeed * Math.tanh(Constants.VisionConstants.kAutotrackIncreaseSpeed * (distanceToGoalMeters-LimitDistance));
    // // double rotationSpeed = MathUtil.applyDeadband(distanceToGoalHorizontalInches, 15) * Math.tanh(Constants.VisionConstants.kAutotrackRotationIncreaseSpeed * distanceToGoalHorizontalInches);
    
    // if (verticalSpeed > 0 && apriltag_id != -1) {
    //   this.swerveSubsystem.move(0.0, verticalSpeed, rotationSpeed, false);
      
    // } else {
    //   this.swerveSubsystem.move(0.0, 0.0, rotationSpeed, false);
    // }

    // SmartDashboard.putNumber("Target id", apriltag_id);
    // SmartDashboard.putNumber("Target Distance", distanceToGoalMeters);
    // SmartDashboard.putNumber("Target Horizontal Distance", distanceToGoalHorizontalInches);
    // SmartDashboard.putNumber("Track rotation speed", rotationSpeed);

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
