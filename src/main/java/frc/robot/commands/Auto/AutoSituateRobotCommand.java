package frc.robot.commands.Auto;

import org.apache.commons.math3.util.FastMath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoSituateRobotCommand extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PIDController drivePIDController = new PIDController(1.5, 0.1, 0.0, 0.01);

  public AutoSituateRobotCommand(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(swerveSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double currentDis = this.swerveSubsystem.getSpeakerDistance();
    final double desiredDis = 4.0;
    final double speed = this.drivePIDController.calculate(currentDis, desiredDis);
    double angle = Units.degreesToRadians(this.swerveSubsystem.getSpeakerAngle());

    double xSpeed = FastMath.cos(angle) * speed;
    double ySpeed = FastMath.sin(angle) * -speed;

    this.shooterSubsystem.toggleSlopeTo(35.0);
    this.swerveSubsystem.situateTowardSpeaker(xSpeed, ySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.swerveSubsystem.stopModules();
    this.shooterSubsystem.stopSlopeToggler();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
