// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoTrackNoteCmd;
import frc.robot.commands.DriveJoystickCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

	private final DriverJoystick driverJoystick = new DriverJoystick(1);
	private final DriverJoystick controllerJoystick = new DriverJoystick(0);

	private final DriveJoystickCmd driverJoystickCommand = new DriveJoystickCmd(swerveSubsystem, driverJoystick);
	private final ShootCmd shootCommand = new ShootCmd(shooterSubsystem, controllerJoystick);

	private final VisionManager visionManager = new VisionManager();
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(driverJoystickCommand);
		this.shooterSubsystem.setDefaultCommand(shootCommand);

		this.autoChooser = AutoBuilder.buildAutoChooser();
		this.configureBindings();
		this.putToDashboard();
	}

	private void putToDashboard() {
		SmartDashboard.putData("SelectAuto", autoChooser);
	}

	private void configureBindings() {
		new JoystickButton(driverJoystick, XboxController.Button.kB.value).whileTrue(new InstantCommand(this.swerveSubsystem::zeroHeading));
		new JoystickButton(driverJoystick, XboxController.Button.kX.value).whileTrue(new RunCommand(this.swerveSubsystem::lockModules, this.swerveSubsystem));
		new JoystickButton(driverJoystick, XboxController.Button.kY.value).whileTrue(new AutoTrackNoteCmd(this.swerveSubsystem, this.visionManager));
	}

	public Command getAutonomousCommand() {
		return this.autoChooser.getSelected();
		//return new AutoTrackNoteCmd(swerveSubsystem, visionManager);
	}
}
