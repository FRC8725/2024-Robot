package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.joysticks.ControllerJoystick;
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
        if (this.joystick.isShootButtonDown()) {
            this.shooterSubsystem.shoot();
        } else {
            this.shooterSubsystem.stopShooters();
        }

        if (this.joystick.isLoadButtonDown()) {
            this.shooterSubsystem.load();
        } else {
            this.shooterSubsystem.stopLoader();
        }

        if (this.joystick.isShooterLifting()) {
            this.shooterSubsystem.lift(this.joystick.getShooterLiftingSpeed());
        } else {
            this.shooterSubsystem.stopLifters();
        }
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
