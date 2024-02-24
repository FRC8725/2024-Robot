package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.joysticks.ControllerJoystick;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

@SuppressWarnings("RedundantMethodOverride")
public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final Supplier<Boolean> canShootSupplier;
    private final ControllerJoystick joystick;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, Supplier<Boolean> canShootSupplier, ControllerJoystick joystick) {
        this.joystick = joystick;
        this.intakeSubsystem = intakeSubsystem;
        this.canShootSupplier = canShootSupplier;
        this.addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        final double lifterDirection = this.joystick.getIntakeLiftDirection();
        if (lifterDirection < 0) this.intakeSubsystem.liftToMax();
        else if (lifterDirection > 0) this.intakeSubsystem.liftToMin();
        else if (this.joystick.getIntakeToAMP()) this.intakeSubsystem.liftTo(IntakeSubsystem.LIFTER_AMP_SETPOINT);
        else this.intakeSubsystem.stopLift();

        if (this.joystick.isIntakeButtonDown()) this.intakeSubsystem.intake();
        else if (this.joystick.isReleaseButtonDown() || this.canShootSupplier.get()) this.intakeSubsystem.release();
        else if (lifterDirection < 0 && !this.intakeSubsystem.isLifterAtMax()) this.intakeSubsystem.adjustNote();
        else this.intakeSubsystem.stopIntake();
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
