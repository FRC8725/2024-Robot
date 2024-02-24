// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

  // TODO: Test if can execute properly
public class AutoAMPCommand extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final Pose2d BLUE_AMP_POSITION = new Pose2d(1.84, 8.07, Rotation2d.fromDegrees(-90.0));
  private final Pose2d RED_AMP_POSITION = new Pose2d(14.701, 8.07, Rotation2d.fromDegrees(-90.0));
  
  private final ProfiledPIDController drivePIDController = new ProfiledPIDController(0.5, 0, 0,
   new TrapezoidProfile.Constraints(SwerveDriveConstants.AUTO_MAX_ROBOT_SPEED, SwerveDriveConstants.AUTO_MAX_ACCELERATION));

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
    boolean isBlue = DriverStation.getAlliance().isPresent() &&
      DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

    Pose2d ampPostion = isBlue ? BLUE_AMP_POSITION : RED_AMP_POSITION;

    double xSpeed = drivePIDController.calculate(this.swerveSubsystem.getRobotPosition().getX(), ampPostion.getX());
    double ySpeed = drivePIDController.calculate(this.swerveSubsystem.getRobotPosition().getY(), ampPostion.getY());

    //.this.swerveSubsystem.situateRobot(0.0, 0.0, AMP_POSITION.getRotation().getDegrees());
    this.intakeSubsystem.liftTo(IntakeSubsystem.LIFTER_AMP_SETPOINT);

    if (this.intakeSubsystem.isLifterAt(IntakeSubsystem.LIFTER_AMP_SETPOINT, 5.0)) {
      this.swerveSubsystem.situateRobot(xSpeed, ySpeed, ampPostion.getRotation().getDegrees());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intakeSubsystem.stopAll();
    this.swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
