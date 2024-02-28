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
import frc.robot.commands.*;
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

    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    private final DriverJoystick driverJoystick = new DriverJoystick();
    private final ControllerJoystick controllerJoystick = new ControllerJoystick();

    private final SendableChooser<Command> autoCommandChooser; // Must NOT be initialized here

    public RobotContainer() {
        this.registerNamedCommands();
        this.setDefaultCommands();
        this.configureBindings();
        this.registerDashboard();

        this.autoCommandChooser = AutoBuilder.buildAutoChooser();
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand(
                "AutoShoot",
                new ParallelRaceGroup(
                        new WaitCommand(5.5),
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
        ); // TODO delete this

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
                                Commands.run(this.intakeSubsystem::liftToMax, this.intakeSubsystem)
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

        this.ledSubsystem.setDefaultCommand(
                new LEDCommand(
                        this.ledSubsystem, 
                        this.visionManager::getNoteGroundDistance,
                        this.shooterSubsystem::getAverageShooterSpeed,
                        this.shooterSubsystem::canShoot,
                        this.controllerJoystick::isShootButtonDown
                ));
    }

    private void configureBindings() {
//        this.driverJoystick.getZeroHeadingTrigger()
//                .onTrue(new InstantCommand(this.swerveSubsystem::resetToMiddlePose, this.swerveSubsystem));
        // this.driverJoystick.getZeroHeadingTrigger()
        //        .whileTrue(Commands.runEnd(this.swerveSubsystem::zeroRobotHeading, this.swerveSubsystem::stopModules, this.swerveSubsystem));
        this.driverJoystick.getNoteTrackingTrigger()
                .whileTrue(new AutoTrackNoteCommand(this.swerveSubsystem, this.visionManager));
        // this.driverJoystick.getAMPTrigger()
        //         .whileTrue(new AutoAMPCommand(this.swerveSubsystem, this.intakeSubsystem));
        // this.driverJoystick.getTestTrigger()
        //         .whileTrue(Commands.runEnd(() -> this.swerveSubsystem.situateRobot(new Pose2d(14.0, 7.0, Rotation2d.fromDegrees(50.0))), this.swerveSubsystem::stopModules, this.swerveSubsystem));
        this.driverJoystick.getStopTrigger()
                .onTrue(Commands.runOnce(this.swerveSubsystem::stopModules, this.swerveSubsystem));

        this.controllerJoystick.getIntakeAMPTrigger()
                .whileTrue(Commands.run(this.intakeSubsystem::releaseAMP, this.intakeSubsystem));
        this.controllerJoystick.getCollectSourceTrigger()
                .whileTrue(Commands.run(() -> {
                        this.shooterSubsystem.collectSource();
                        this.intakeSubsystem.executeIntake();
                }, this.shooterSubsystem, this.intakeSubsystem));
    }

    public void teleopPeriodic() {
        if (this.visionManager.hasTagTarget()) {
            this.swerveSubsystem.calibratePose(this.visionManager.getVisionRobotPose());
        }
    }

    public Command getAutonomousCommand() {
        return this.autoCommandChooser.getSelected();
    }

    public void putDashboard() {
    }

    @Override
    public void putDashboardOnce() {
        SmartDashboard.putData("SelectAuto", this.autoCommandChooser);
    }
}
