package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.joysticks.ControllerJoystick;
import frc.robot.subsystems.IntakeSubsystem;

// TODO: Transform it into PIDControl :D:)
@SuppressWarnings("RedundantMethodOverride")
public class IntakeCommand extends Command {
    private static final double LIFTER_MAX_TARGET = 173.0;
    private static final double LIFTER_MIN_TARGET = 13.0;
    private final IntakeSubsystem intakeSubsystem;
    private final ControllerJoystick joystick;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, ControllerJoystick joystick) {
        this.joystick = joystick;
        this.intakeSubsystem = intakeSubsystem;
        this.addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        final double lifterDirection = this.joystick.getIntakeLiftDirection();
        if (lifterDirection < 0) this.intakeSubsystem.liftTo(LIFTER_MAX_TARGET);
        else if (lifterDirection > 0) this.intakeSubsystem.liftTo(LIFTER_MIN_TARGET);
        else if (this.joystick.getRightBumper()) this.intakeSubsystem.liftTo(103.86);
        else this.intakeSubsystem.stopLift();

        if (this.joystick.isIntakeButtonDown()) this.intakeSubsystem.intake();
        else if (this.joystick.isReleaseButtonDown()) this.intakeSubsystem.release();
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
