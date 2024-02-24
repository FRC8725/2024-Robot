package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.helpers.TidiedUp;
import frc.robot.joysticks.ControllerJoystick;
import frc.robot.subsystems.TelescopeSubsystem;

@TidiedUp
@SuppressWarnings("RedundantMethodOverride")
public class TelescopeCommand extends Command {
    private final TelescopeSubsystem telescopeSubsystem;
    private final ControllerJoystick controller;

    public TelescopeCommand(TelescopeSubsystem telescopeSubsystem, ControllerJoystick controller) {
        this.telescopeSubsystem = telescopeSubsystem;
        this.controller = controller;

        this.addRequirements(telescopeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (this.controller.isTelescoping()) {
            this.telescopeSubsystem.move(this.controller.getTelescopeDirection() == 1);
        } else {
            this.telescopeSubsystem.stopAll();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.telescopeSubsystem.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
