package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

@SuppressWarnings("RedundantMethodOverride")
public class ElevatorCmd extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final XboxController controller;

    public ElevatorCmd(ElevatorSubsystem elevatorSubsystem, XboxController controller) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.controller = controller;
        this.addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (controller.getYButton()) this.elevatorSubsystem.move(true);
        else if (controller.getAButton()) this.elevatorSubsystem.move(false);
        else this.elevatorSubsystem.stop();

    }

    @Override
    public void end(boolean interrupted) {
        this.elevatorSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
