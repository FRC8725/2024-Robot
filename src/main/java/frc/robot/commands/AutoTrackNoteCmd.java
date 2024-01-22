// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

public class AutoTrackNoteCmd extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final PIDController autoTrackPidController = new PIDController(Constants.VisionConstants.kAutoTrackYP, 0, 0);
  private final PIDController autoTrackThetaPidController = new PIDController(Constants.VisionConstants.kAutoTrackThetaP, 0, 0);
  private final VisionManager visionManager;

  /** Creates a new AutoTrackCmd. */
  public AutoTrackNoteCmd(SwerveSubsystem swerveSubsystem, VisionManager visionManager) {
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
    double note_distance = this.visionManager.getNoteDistance();
    double note_horizontal = this.visionManager.getNoteHorizontal();

    double target_distance = 120.0;
    double target_horizontal = 9.2;

    double vertical_speed = autoTrackPidController.calculate(target_distance, note_distance);
    double rotation_speed = MathUtil.applyDeadband(autoTrackThetaPidController.calculate(target_horizontal, note_horizontal), 0.15);
    
    SmartDashboard.putNumber("Vertical_speed", vertical_speed);
    SmartDashboard.putNumber("Rotation_speed", rotation_speed);

    if (this.visionManager.hasNoteTarget() && vertical_speed > 0) this.swerveSubsystem.move(0.0, vertical_speed, rotation_speed, false);
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
