package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.helpers.IDashboardProvider;
import frc.robot.commands.*;
import frc.robot.joysticks.ControllerJoystick;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.subsystems.*;

public class RobotContainer implements IDashboardProvider {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final TelescopeSubsystem elevatorSubsystem = new TelescopeSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final DriverJoystick driverJoystick = new DriverJoystick();
    private final ControllerJoystick controllerJoystick = new ControllerJoystick();
    private final VisionManager visionManager = new VisionManager();
    private final SendableChooser<Command> autoCommandChooser;

    public RobotContainer() {
        this.autoCommandChooser = AutoBuilder.buildAutoChooser();
        this.setDefaultCommands();
        this.configureBindings();
        this.registerDashboard();
    }

    private void setDefaultCommands() {
        this.swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(this.swerveSubsystem, this.driverJoystick));
        this.shooterSubsystem.setDefaultCommand(new ShootCommand(this.shooterSubsystem, this.controllerJoystick));
        this.intakeSubsystem.setDefaultCommand(new TelescopeCommand(this.elevatorSubsystem, this.controllerJoystick));
        this.intakeSubsystem.setDefaultCommand(new IntakeCommand(this.intakeSubsystem, this.controllerJoystick));
    }

    private void configureBindings() {
        this.driverJoystick.getZeroHeadingTrigger().onTrue(new InstantCommand(this.swerveSubsystem::resetGyro, this.swerveSubsystem));
        this.driverJoystick.getModuleLockingTrigger().whileTrue(new RunCommand(this.swerveSubsystem::lockModules, this.swerveSubsystem));
        this.driverJoystick.getNoteTrackingTrigger().whileTrue(new AutoTrackNoteCommand(this.swerveSubsystem, this.visionManager));
    }

    public Command getAutonomousCommand() {
        return this.autoCommandChooser.getSelected();
        // return new AutoTrackNoteCmd(swerveSubsystem, visionManager);
    }

    @Override
    public void putDashboard() {
    }

    @Override
    public void putDashboardOnce() {
        SmartDashboard.putData("SelectAuto", this.autoCommandChooser);
    }
}
