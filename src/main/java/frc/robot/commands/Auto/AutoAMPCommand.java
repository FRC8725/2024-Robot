// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

  // TODO: Test if can execute properly
public class AutoAMPCommand extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private final Pose2d AMP_POSITION = new Pose2d(1.93, 7.77, Rotation2d.fromDegrees(-90.0));
  private final PIDController drivePIDController = new PIDController(0.5, 0.0, 0.0);
  /** Creates a new AutoAMPCommand. */
  public AutoAMPCommand(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(swerveSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = drivePIDController.calculate(this.swerveSubsystem.getRobotPosition().getX(), AMP_POSITION.getX());
    double ySpeed = drivePIDController.calculate(this.swerveSubsystem.getRobotPosition().getY(), AMP_POSITION.getY());

    this.intakeSubsystem.liftTo(IntakeSubsystem.LIFTER_AMP_SETPOINT);

    if (this.intakeSubsystem.isLifterAt(IntakeSubsystem.LIFTER_AMP_SETPOINT, 5.0)) {
      this.swerveSubsystem.situateRobot(xSpeed, ySpeed, AMP_POSITION.getRotation().getRadians());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
