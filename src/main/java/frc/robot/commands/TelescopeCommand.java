package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.joysticks.ControllerJoystick;
import frc.robot.subsystems.TelescopeSubsystem;

@SuppressWarnings("RedundantMethodOverride")
public class TelescopeCommand extends Command {
    private final TelescopeSubsystem subsystem;
    private final ControllerJoystick controller;

    public TelescopeCommand(TelescopeSubsystem subsystem, ControllerJoystick controller) {
        this.subsystem = subsystem;
        this.controller = controller;
        this.addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (this.controller.isTelescoping()) {
            this.subsystem.move(this.controller.getTelescopeDirection() == 1);
        } else {
            this.subsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
