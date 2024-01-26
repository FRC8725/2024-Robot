package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ModuleTalonFX;
import frc.lib.helpers.IDashboardProvider;
import frc.robot.constants.RobotCANPorts;

public class ShooterSubsystem extends SubsystemBase implements IDashboardProvider {
    private static final double SHOOT_SPEED = 1;
    private static final double LOAD_SPEED = 0.9;
    private static final double LIFT_COEFFICIENT = 0.07;
    private final ModuleTalonFX rightShootMotor = new ModuleTalonFX(RobotCANPorts.RIGHT_SHOOTER.get());
    private final ModuleTalonFX leftShootMotor = new ModuleTalonFX(RobotCANPorts.LEFT_SHOOTER.get());
    private final ModuleTalonFX loadMotor = new ModuleTalonFX(RobotCANPorts.ROLLER.get());
    private final ModuleTalonFX rightLiftMotor = new ModuleTalonFX(RobotCANPorts.RIGHT_SHOOTER_LIFTER.get());
    private final ModuleTalonFX leftLiftMotor = new ModuleTalonFX(RobotCANPorts.LEFT_SHOOTER_LIFTER.get());
    private final DutyCycleEncoder liftEncoder = new DutyCycleEncoder(1); // Shoot rotation without offset 0.98
    private final PIDController liftPID = new PIDController(0.1, 0, 0);

    public ShooterSubsystem() {
        this.rightShootMotor.setInverted(false);
        this.leftShootMotor.setInverted(true);
        this.loadMotor.setInverted(false);
        this.rightLiftMotor.setInverted(true);
        this.leftLiftMotor.setInverted(false);

        this.rightLiftMotor.setNeutralMode(NeutralModeValue.Brake);
        this.leftLiftMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void shoot() {
        this.rightShootMotor.set(SHOOT_SPEED);
        this.leftShootMotor.set(SHOOT_SPEED);
        // loadMotor.set(LoadSpeed);
    }

    public void stopShooters() {
        this.rightShootMotor.stopMotor();
        this.leftShootMotor.stopMotor();
    }

    public void load() {
        this.loadMotor.set(LOAD_SPEED);
    }

    public void stopLoader() {
        this.loadMotor.stopMotor();
    }

    public void lift(double speed) {
        this.rightLiftMotor.set(speed * LIFT_COEFFICIENT);
        this.leftLiftMotor.set(speed * LIFT_COEFFICIENT);
    }

    public void liftTo(double setpoint) {
        this.lift(this.liftPID.calculate(this.liftEncoder.getAbsolutePosition(), setpoint));
    }

    public void stopLifters() {
        this.rightLiftMotor.stopMotor();
        this.leftLiftMotor.stopMotor();
    }

    public void stopAll() {
        this.stopShooters();
        this.stopLifters();
        this.stopLoader();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Shooter Position", liftEncoder.getAbsolutePosition());
        double mean = (this.leftShootMotor.getVelocity().getValue() + this.rightShootMotor.getVelocity().getValue()) / 2.0;
        double error = Math.abs(this.leftShootMotor.getVelocity().getValue() - this.rightShootMotor.getVelocity().getValue());
        SmartDashboard.putNumber("Shooter Speed", mean * Math.PI * Units.inchesToMeters(4));
        SmartDashboard.putNumber("Shooter Error", error * Math.PI * Units.inchesToMeters(4));
        SmartDashboard.putNumber("Loader Speed", loadMotor.getVelocity().getValue() * Math.PI * 0.0485 / 3);
    }

    @Override
    public void putDashboardOnce() {
    }
}
