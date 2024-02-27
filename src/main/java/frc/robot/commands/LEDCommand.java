package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionManager;

@SuppressWarnings("RedundantMethodOverride")
public class LEDCommand extends Command {
	private final Supplier<Boolean> noNoteSupplier;
	private final Supplier<Double> shooterSpeed;
	private final Supplier<Boolean> canShootSupplier;
	public final LEDSubsystem ledSubsystem;

  	public LEDCommand(LEDSubsystem ledSubsystem, Supplier<Boolean> hasNoteSupplier, Supplier<Double> shooterSpeed, Supplier<Boolean> canShootSupplier){
		this.ledSubsystem = ledSubsystem;
		this.noNoteSupplier = hasNoteSupplier;
		this.shooterSpeed = shooterSpeed;
		this.canShootSupplier = canShootSupplier;

		addRequirements(this.ledSubsystem);
  	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		if (this.canShootSupplier.get()) {
			this.ledSubsystem.setColor(Color.kGreen);
		}else if (this.shooterSpeed.get() != 0) {
			this.ledSubsystem.setIdleMode(false);
			this.ledSubsystem.setColorWithPercentage(Color.kOrangeRed, Color.kBlack, this.shooterSpeed.get() / 0.8);
		} else if (!this.noNoteSupplier.get()) {
			this.ledSubsystem.setIdleMode(false);
			this.ledSubsystem.setColor(Color.kOrangeRed);
		} else {
			this.ledSubsystem.setIdleMode(true);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}
