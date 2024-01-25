// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


// TODO: Transform it into PIDControl :D:)
public class IntakeCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final XboxController controller;
  /** Creates a new IntakeCmd. */
  public IntakeCmd(IntakeSubsystem intakeSubsystem, XboxController controller) {
    this.controller = controller;
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
    if(controller.getLeftTriggerAxis() > 0.1) this.intakeSubsystem.lift(-controller.getLeftTriggerAxis());
    else if(controller.getRightTriggerAxis() > 0.1) this.intakeSubsystem.lift(controller.getRightTriggerAxis());
    else this.intakeSubsystem.stopLift();

    if (controller.getAButton()) this.intakeSubsystem.intake();
    else this.intakeSubsystem.stopIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intakeSubsystem.stopIntake();
    this.intakeSubsystem.stopLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
