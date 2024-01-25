package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

// TODO: Transform it into PIDControl :D:)
public class IntakeCmd extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final XboxController controller;

    public IntakeCmd(IntakeSubsystem intakeSubsystem, XboxController controller) {
        this.controller = controller;
        this.intakeSubsystem = intakeSubsystem;
        this.addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (controller.getLeftTriggerAxis() > 0.1) this.intakeSubsystem.lift(-controller.getLeftTriggerAxis());
        else if (controller.getRightTriggerAxis() > 0.1) this.intakeSubsystem.lift(controller.getRightTriggerAxis());
        else this.intakeSubsystem.stopLift();

        if (controller.getAButton()) this.intakeSubsystem.intake();
        else this.intakeSubsystem.stopIntake();
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.stopIntake();
        this.intakeSubsystem.stopLift();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
