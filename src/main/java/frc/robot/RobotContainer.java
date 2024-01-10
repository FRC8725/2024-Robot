// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoTrackCmd;
import frc.robot.commands.DriveJoystickCmd;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final DriverJoystick driverJoystick = new DriverJoystick(0);
    private final DriveJoystickCmd driverJoystickCommand = new DriveJoystickCmd(swerveSubsystem, driverJoystick);
    private final VisionManager visionManager = new VisionManager();

  public RobotContainer() {
    this.swerveSubsystem.setDefaultCommand(driverJoystickCommand);
    this.configureBindings();
    this.putToDashboard();
  }


  private void putToDashboard() {

  }

  private void configureBindings() {
      new JoystickButton(driverJoystick, XboxController.Button.kB.value).whileTrue(new InstantCommand(this.swerveSubsystem::zeroHeading));
      new JoystickButton(driverJoystick, XboxController.Button.kY.value).whileTrue(new AutoTrackCmd(this.swerveSubsystem, this.visionManager));
  }

  public Command getAutonomousCommand() {
      return new AutoTrackCmd(swerveSubsystem, visionManager);
      // return new Auto(swerveSubsystem);
        
  }

}
