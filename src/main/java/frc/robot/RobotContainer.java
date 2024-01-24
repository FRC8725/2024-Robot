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
import frc.robot.commands.AutoTrackNoteCmd;
import frc.robot.commands.ElevatorCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.joysticks.SwerveJoystick;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

public class RobotContainer implements IDashboardProvider {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final SwerveJoystick swerveJoystick = new SwerveJoystick(1);
    private final DriverJoystick controllerJoystick = new DriverJoystick(0);
    private final VisionManager visionManager = new VisionManager();
    private final SendableChooser<Command> autoCommandChooser;

    public RobotContainer() {
        this.swerveJoystick.setDefaultCommand(this.swerveSubsystem);
        ShootCmd shootCommand = new ShootCmd(shooterSubsystem, controllerJoystick);
        ElevatorCmd elevatorCmd = new ElevatorCmd(elevatorSubsystem, controllerJoystick);

        this.shooterSubsystem.setDefaultCommand(shootCommand);
        this.elevatorSubsystem.setDefaultCommand(elevatorCmd);
        
        this.autoCommandChooser = AutoBuilder.buildAutoChooser();
        this.configureBindings();
        this.registerDashboard();
    }

    public void setSwerveStart() {
        this.swerveSubsystem.setStart();
    }

    public void resetSwerveStart() {
        this.swerveSubsystem.resetStart();
    }

    private void configureBindings() {
        new JoystickButton(swerveJoystick, XboxController.Button.kB.value).whileTrue(new InstantCommand(this.swerveSubsystem::zeroHeading));
        new JoystickButton(swerveJoystick, XboxController.Button.kX.value).whileTrue(new RunCommand(this.swerveSubsystem::lockModules, this.swerveSubsystem));
        new JoystickButton(swerveJoystick, XboxController.Button.kY.value).whileTrue(new AutoTrackNoteCmd(this.swerveSubsystem, this.visionManager));
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
