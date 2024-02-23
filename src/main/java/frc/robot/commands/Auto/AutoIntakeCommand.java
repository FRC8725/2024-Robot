// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

  // TODO: Test if can execute properly
public class AutoIntakeCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  /** Creates a new AutoIntakeCommand. */
  public AutoIntakeCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.intakeSubsystem.liftTo(IntakeSubsystem.LIFTER_MIN_SETPOINT);
    if (this.intakeSubsystem.isLifterAt(IntakeSubsystem.LIFTER_MIN_SETPOINT, 5.0)) {
      this.intakeSubsystem.intake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    andThen(Commands.runOnce(this.intakeSubsystem::stopAll, this.intakeSubsystem));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
