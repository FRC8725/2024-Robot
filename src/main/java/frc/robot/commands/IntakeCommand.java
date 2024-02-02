package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.joysticks.ControllerJoystick;
import frc.robot.subsystems.IntakeSubsystem;

// TODO: Transform it into PIDControl :D:)
@SuppressWarnings("RedundantMethodOverride")
public class IntakeCommand extends Command {
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
        final double angleTogglerDirection = this.joystick.getIntakeLiftDirection();
        if (angleTogglerDirection != 0) this.intakeSubsystem.lift(angleTogglerDirection);
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
