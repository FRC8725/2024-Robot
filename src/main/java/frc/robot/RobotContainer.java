package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.helpers.IDashboardProvider;
import frc.robot.commands.*;
import frc.robot.joysticks.ControllerJoystick;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

public class RobotContainer implements IDashboardProvider {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final DriverJoystick driverJoystick = new DriverJoystick(1);
    private final ControllerJoystick controllerJoystick = new ControllerJoystick(0);
    private final VisionManager visionManager = new VisionManager();
    private final SendableChooser<Command> autoCommandChooser;

    public RobotContainer() {
        this.autoCommandChooser = AutoBuilder.buildAutoChooser();
        this.setDefaultCommands();
        this.configureBindings();
        this.registerDashboard();
    }

    private void setDefaultCommands() {
        this.swerveSubsystem.setDefaultCommand(new SwerveDriveCmd(this.swerveSubsystem, this.driverJoystick));
        this.shooterSubsystem.setDefaultCommand(new ShootCmd(this.shooterSubsystem, this.controllerJoystick));
        this.elevatorSubsystem.setDefaultCommand(new ElevatorCmd(this.elevatorSubsystem, this.controllerJoystick));
        this.intakeSubsystem.setDefaultCommand(new IntakeCmd(this.intakeSubsystem, this.controllerJoystick));
    }

    public void setSwerveStart() {
        this.swerveSubsystem.setStart();
    }

    public void resetSwerveStart() {
        this.swerveSubsystem.resetStart();
    }

    private void configureBindings() {
        new JoystickButton(driverJoystick, XboxController.Button.kB.value).whileTrue(new InstantCommand(this.swerveSubsystem::zeroHeading));
        new JoystickButton(driverJoystick, XboxController.Button.kX.value).whileTrue(new RunCommand(this.swerveSubsystem::lockModules, this.swerveSubsystem));
        new JoystickButton(driverJoystick, XboxController.Button.kY.value).whileTrue(new AutoTrackNoteCmd(this.swerveSubsystem, this.visionManager));
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
