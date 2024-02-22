package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.joysticks.ControllerJoystick;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

@SuppressWarnings("RedundantMethodOverride")
public class ShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final ControllerJoystick joystick;

    public ShootCommand(ShooterSubsystem shooterSubsystem, ControllerJoystick joystick) {
        this.shooterSubsystem = shooterSubsystem;
        this.joystick = joystick;
        this.addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (this.joystick.isShootButtonDown()) this.shooterSubsystem.shoot();
        else this.shooterSubsystem.stopShooters();

        final double slopeTogglerDirection = this.joystick.getSlopeTogglerDirection();
        if (slopeTogglerDirection != 0) this.shooterSubsystem.toggleSlope(slopeTogglerDirection);
        else this.shooterSubsystem.stopSlopeToggler();
    }

    @Override
    public void end(boolean interrupted) {
        this.shooterSubsystem.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
