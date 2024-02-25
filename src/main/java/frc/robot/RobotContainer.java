package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.helpers.IDashboardProvider;
import frc.lib.helpers.TidiedUp;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TelescopeCommand;
import frc.robot.commands.auto.AutoAMPCommand;
import frc.robot.commands.auto.AutoTrackNoteCommand;
import frc.robot.joysticks.ControllerJoystick;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.subsystems.*;

@TidiedUp
public class RobotContainer implements IDashboardProvider {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final VisionManager visionManager = new VisionManager();
    private final DriverJoystick driverJoystick = new DriverJoystick();
    private final ControllerJoystick controllerJoystick = new ControllerJoystick();
    private final SendableChooser<Command> autoCommandChooser = AutoBuilder.buildAutoChooser();

    public RobotContainer() {
        this.registerNamedCommands();
        this.setDefaultCommands();
        this.configureBindings();
        this.registerDashboard();
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand(
                "AutoShoot",
                new ParallelDeadlineGroup(new WaitCommand(5.5),
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(new WaitCommand(1.5),
                                        Commands.run(this.shooterSubsystem::executeShooter, this.shooterSubsystem),
                                        new SequentialCommandGroup(
                                                Commands.waitUntil(this.shooterSubsystem::canShoot),
                                                Commands.run(this.intakeSubsystem::releaseIntake, this.intakeSubsystem)
                                        )
                                ),

                                Commands.runOnce(this.shooterSubsystem::stopAll, this.shooterSubsystem),
                                Commands.runOnce(this.intakeSubsystem::stopIntake, this.intakeSubsystem)
                        )
                )
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

        NamedCommands.registerCommand(
                "ToIntakePoint",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(Commands.waitUntil(this.intakeSubsystem::isLifterAtMin),
                                Commands.run(this.intakeSubsystem::liftToMin, this.intakeSubsystem)
                        ),

                        Commands.run(this.intakeSubsystem::executeIntake, this.intakeSubsystem)
                )
        );

        NamedCommands.registerCommand(
                "StopAndLiftIntake",
                new SequentialCommandGroup(
                        Commands.runOnce(this.intakeSubsystem::stopAll, this.intakeSubsystem),
                        new ParallelDeadlineGroup(Commands.waitUntil(this.intakeSubsystem::isLifterAtMax),
                                Commands.run(this.intakeSubsystem::liftToMax, this.intakeSubsystem),
                                Commands.run(this.intakeSubsystem::fineTuneNote, this.intakeSubsystem)
                        ),
                        Commands.runOnce(this.intakeSubsystem::stopAll, this.intakeSubsystem)
                )
        );
    }

    private void setDefaultCommands() {
        this.swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(this.swerveSubsystem, this.driverJoystick));
        this.shooterSubsystem.setDefaultCommand(new ShootCommand(this.shooterSubsystem, this.controllerJoystick));
        this.telescopeSubsystem.setDefaultCommand(new TelescopeCommand(this.telescopeSubsystem, this.controllerJoystick));
        this.intakeSubsystem.setDefaultCommand(new IntakeCommand(this.intakeSubsystem,
                this.shooterSubsystem::canShoot, this.controllerJoystick));
    }

    private void configureBindings() {
        this.driverJoystick.getZeroHeadingTrigger()
                .onTrue(new InstantCommand(this.swerveSubsystem::resetToMiddlePose, this.swerveSubsystem));
        // this.driverJoystick.getNoteTrackingTrigger()
        //         .whileTrue(new AutoTrackNoteCommand(this.swerveSubsystem, this.visionManager));
        this.driverJoystick.getAMPTrigger()
                .whileTrue(new AutoAMPCommand(this.swerveSubsystem, this.intakeSubsystem));
        this.driverJoystick.getTestTrigger()
                .whileTrue(Commands.run(() -> this.swerveSubsystem.situateRobot(new Pose2d(14.17, 1.79, new Rotation2d(0.69))), this.swerveSubsystem));
    }

    public void teleopPeriodic() {
        if (this.visionManager.hasTagTarget()) {
            this.swerveSubsystem.calibratePose(this.visionManager.getVisionRobotPose());
        }
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
