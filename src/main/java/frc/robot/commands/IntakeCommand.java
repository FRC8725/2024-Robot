package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.joysticks.ControllerJoystick;
import frc.robot.subsystems.IntakeSubsystem;

@SuppressWarnings("RedundantMethodOverride")
public class IntakeCommand extends Command {
    private static final double LIFTER_MAX_SETPOINT = 170.0;
    private static final double LIFTER_MIN_SETPOINT = 11.0;
    private static final double LIFTER_AMP_SETPOINT = 103.86; //5cm to AMP, note to bumper, 

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
        if (lifterDirection < 0) this.intakeSubsystem.liftTo(LIFTER_MAX_SETPOINT);
        else if (lifterDirection > 0) this.intakeSubsystem.liftTo(LIFTER_MIN_SETPOINT);
        else if (this.joystick.getIntakeToAMP()) this.intakeSubsystem.liftTo(LIFTER_AMP_SETPOINT);
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
