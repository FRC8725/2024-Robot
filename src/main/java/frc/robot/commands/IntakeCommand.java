package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.helpers.TidiedUp;
import frc.robot.joysticks.ControllerJoystick;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.Supplier;

@TidiedUp
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
        final boolean flag = (lifterDirection < 0) || this.joystick.getIntakeToAMP();

        if (lifterDirection < 0) this.intakeSubsystem.liftToMax();
        else if (lifterDirection > 0) this.intakeSubsystem.liftToMin();
        else if (this.joystick.getIntakeToAMP()) this.intakeSubsystem.liftToAMP();
        else this.intakeSubsystem.stopLifters();

        if (this.joystick.isIntakeButtonDown()) this.intakeSubsystem.executeIntake();
        else if (this.joystick.isReleaseButtonDown() || this.canShootSupplier.get()) 
                this.intakeSubsystem.releaseIntake();
        else if (!flag) this.intakeSubsystem.stopIntake();

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
