package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionManager;

@SuppressWarnings("RedundantMethodOverride")
public class LEDCommand extends Command {

	private final Supplier<Double> noteDistanceSupplier;
	private final Supplier<Double> shooterSpeedSupplier;
	private final Supplier<Boolean> canShootSupplier;
	private final Supplier<Boolean> isShootButtonDown;

	private boolean shootFlag = false;
	private int lastLEDButterIndex = 0;
	private int count = 0;

	public final LEDSubsystem ledSubsystem;
  	/** Creates a new LEDCommand. */
  	public LEDCommand(LEDSubsystem ledSubsystem, Supplier<Double> noteDistanceSupplier, Supplier<Double> shooterSpeedSupplier, Supplier<Boolean> canShootSupplier, Supplier<Boolean> isShootButtonDown){
		this.ledSubsystem = ledSubsystem;
		this.noteDistanceSupplier = noteDistanceSupplier;
		this.shooterSpeedSupplier = shooterSpeedSupplier;
		this.canShootSupplier = canShootSupplier;
		this.isShootButtonDown = isShootButtonDown;

		addRequirements(this.ledSubsystem);
  	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		if (!this.isShootButtonDown.get()) this.shootFlag = false;

		if (this.canShootSupplier.get() || this.shootFlag) {
			this.shootFlag = true;
			this.ledSubsystem.setIdleMode(false);
			this.ledSubsystem.setAllColor(Color.kRed);

		}else if (this.shooterSpeedSupplier.get() >= 0.1) {
			this.ledSubsystem.setIdleMode(false);
			this.ledSubsystem.setColorWithPercentage(Color.kOrangeRed, Color.kBlack, 1 - (this.shooterSpeedSupplier.get() / 0.8));

		} else if (this.shooterSpeedSupplier.get() <= -0.1) {
			this.ledSubsystem.setIdleMode(false);
			this.ledSubsystem.setColorInRange(Color.kBlack, Color.kDarkViolet, lastLEDButterIndex-4, lastLEDButterIndex+4);
			if (count % 2 == 0) lastLEDButterIndex++;
			if (lastLEDButterIndex >= LEDSubsystem.TELESCOPE_LED_BUFFER_EACH_LENGTH) lastLEDButterIndex = 0;
			if (count >= 255) count = 0;
			count++;

		} else if (this.noteDistanceSupplier.get() >= 0.4) {
			this.ledSubsystem.setIdleMode(false);
			this.ledSubsystem.setColorWithPercentage(Color.kOrangeRed, Color.kBlack, (this.noteDistanceSupplier.get() - 0.4) / VisionManager.MAX_NOTE_DISTANCE);

		} else {
			this.ledSubsystem.setIdleMode(true);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		this.ledSubsystem.setIdleMode(true);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
