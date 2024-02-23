

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoShootCommand extends ParallelDeadlineGroup {
	private final SwerveSubsystem swerveSubsystem;
	private final IntakeSubsystem intakeSubsystem;
	private final ShooterSubsystem shooterSubsystem;

	public AutoShootCommand(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
		super(new WaitCommand(5.5)); // Deadline

		this.swerveSubsystem = swerveSubsystem;
		this.intakeSubsystem = intakeSubsystem;
		this.shooterSubsystem = shooterSubsystem;

		addCommands(
			new SequentialCommandGroup(
				new ParallelDeadlineGroup(new WaitCommand(1.0),
					new AutoAimCommand(this.swerveSubsystem, this.shooterSubsystem)
				),

				new ParallelDeadlineGroup(new WaitCommand(1.5),
					Commands.run(this.shooterSubsystem::shoot, this.shooterSubsystem),
					new SequentialCommandGroup(
						Commands.waitUntil(this.shooterSubsystem::canShoot),
						Commands.run(this.intakeSubsystem::release, this.intakeSubsystem)
					)
				),

				Commands.runOnce(this.shooterSubsystem::stopShooters, this.shooterSubsystem),
				Commands.runOnce(this.intakeSubsystem::stopIntake, this.intakeSubsystem)
			)
		);
	}
}

// return new SequentialCommandGroup(
//     this.autoCommandChooser.getSelected(),
//     new ParallelDeadlineGroup(new WaitCommand(1.0),
//         Commands.run(() -> this.swerveSubsystem.drive(0.6, 0.0, 0.0, true), this.swerveSubsystem)
//         ),
//     new ParallelDeadlineGroup(new WaitCommand(5.0), new SequentialCommandGroup(
//         new ParallelDeadlineGroup(new WaitCommand(2.0),
//             new AutoAimCommand(swerveSubsystem, shooterSubsystem, visionManager)),
//         new ParallelCommandGroup(
//             Commands.run(this.shooterSubsystem::shoot, this.shooterSubsystem),
//             new SequentialCommandGroup(
//                 new WaitCommand(1.5),
//                 Commands.run(this.intakeSubsystem::release, this.intakeSubsystem)
//             )
//         )
//     ))
// );
