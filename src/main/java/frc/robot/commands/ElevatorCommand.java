package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.joysticks.ControllerJoystick;
import frc.robot.subsystems.ElevatorSubsystem;

@SuppressWarnings("RedundantMethodOverride")
public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final ControllerJoystick joystick;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, ControllerJoystick joystick) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.joystick = joystick;
        this.addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (this.joystick.isElevating()) {
            this.elevatorSubsystem.move(this.joystick.getElevatorDirection() == 1);
        } else {
            this.elevatorSubsystem.stop();
        }
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
