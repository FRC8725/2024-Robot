// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCmd extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final XboxController controller;
  /** Creates a new ElevatorCmd. */
  public ElevatorCmd(ElevatorSubsystem elevatorSubsystem, XboxController controller) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.controller = controller;

    addRequirements(this.elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (controller.getYButton()) this.elevatorSubsystem.move(true);
      else if (controller.getAButton()) this.elevatorSubsystem.move(false);
      else this.elevatorSubsystem.stop();

      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.elevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
