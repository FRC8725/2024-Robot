package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.helpers.IDashboardProvider;
import frc.robot.commands.*;
import frc.robot.joysticks.ControllerJoystick;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.subsystems.*;

public class RobotContainer implements IDashboardProvider {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final DriverJoystick driverJoystick = new DriverJoystick();
    private final ControllerJoystick controllerJoystick = new ControllerJoystick();
    private final VisionManager visionManager = new VisionManager();
    private final SendableChooser<Command> autoCommandChooser;

    public RobotContainer() {
        NamedCommands.registerCommand(
            "AutoShoot",
            new AutoShootCommand(this.swerveSubsystem, this.intakeSubsystem, this.shooterSubsystem)
        );

        NamedCommands.registerCommand(
            "StopSwerve", 
            Commands.runOnce(this.swerveSubsystem::stopModules, this.swerveSubsystem)
        );

        NamedCommands.registerCommand(
            "ZeroRobotHeading",
            new ParallelDeadlineGroup(new WaitCommand(2.0),
                Commands.run(this.swerveSubsystem::zeroRobotHeading, this.swerveSubsystem)
            )
        );

        this.autoCommandChooser = AutoBuilder.buildAutoChooser();

        this.setDefaultCommands();
        this.configureBindings();
        this.registerDashboard();
    }

    public void robotPeriodic() {
        if (this.visionManager.hasTagTarget()) {
            this.swerveSubsystem.adjustPoseEstimator(this.visionManager.getVisionRobotPose());
        }
    }

    private void setDefaultCommands() {
        this.swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(this.swerveSubsystem, this.driverJoystick));
        this.shooterSubsystem.setDefaultCommand(new ShootCommand(this.shooterSubsystem, this.controllerJoystick));
        this.telescopeSubsystem.setDefaultCommand(new TelescopeCommand(this.telescopeSubsystem, this.controllerJoystick));
        this.intakeSubsystem.setDefaultCommand(new IntakeCommand(this.intakeSubsystem, this.controllerJoystick));
    }

    private void configureBindings() {
        this.driverJoystick.getZeroHeadingTrigger().onTrue(new InstantCommand(this.swerveSubsystem::resetGyro, this.swerveSubsystem));
        this.driverJoystick.getModuleLockingTrigger().whileTrue(new RunCommand(this.swerveSubsystem::lockModules, this.swerveSubsystem));
        this.driverJoystick.getSpeakerAimingTrigger().whileTrue(new AutoAimCommand(this.swerveSubsystem, this.shooterSubsystem));
        this.driverJoystick.getNoteTrackingTrigger().whileTrue(new AutoTrackNoteCommand(swerveSubsystem, visionManager));
    }
    public Command getAutonomousCommand() {
        return this.autoCommandChooser.getSelected();
    }

    @Override
    public void putDashboard() {
    }

    @Override
    public void putDashboardOnce() {
        SmartDashboard.putData("SelectAuto", this.autoCommandChooser);
    }
}
