// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final XboxController controller;
  /** Creates a new ShootCmd. */
  public ShootCommand(ShooterSubsystem shooterSubsystem, XboxController controller) {
    this.shooterSubsystem = shooterSubsystem;
    this.controller = controller;
    addRequirements(this.shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.getLeftBumper()) this.shooterSubsystem.shoot();
    else this.shooterSubsystem.stopShoot(); 

    if (controller.getRightBumper()) this.shooterSubsystem.load();
    else this.shooterSubsystem.stopLoad();

    if(controller.getLeftTriggerAxis() > 0.1) this.shooterSubsystem.lift(-controller.getLeftTriggerAxis());
    else if(controller.getRightTriggerAxis() > 0.1) this.shooterSubsystem.lift(controller.getRightTriggerAxis());
    else this.shooterSubsystem.stopLift();

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.stopShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
