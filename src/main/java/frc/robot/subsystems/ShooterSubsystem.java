package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ModuleTalonFX;
import frc.robot.constants.RobotCANPorts;

public class ShooterSubsystem extends SubsystemBase {
    private final ModuleTalonFX rightShootMotor = new ModuleTalonFX(RobotCANPorts.RIGHT_SHOOTER.get());
    private final ModuleTalonFX leftShootMotor = new ModuleTalonFX(RobotCANPorts.LEFT_SHOOTER.get());
    private final ModuleTalonFX loadMotor = new ModuleTalonFX(RobotCANPorts.ROLLER.get());
    private final ModuleTalonFX rightLiftMotor = new ModuleTalonFX(RobotCANPorts.RIGHT_SHOOTER_LIFTER.get());
    private final ModuleTalonFX leftLiftMotor = new ModuleTalonFX(RobotCANPorts.LEFT_SHOOTER_LIFTER.get());
    
    private final DutyCycleEncoder liftEncoder = new DutyCycleEncoder(1); // Shoot rotation without offset 0.98
    private final PIDController liftPID = new PIDController(0.1, 0, 0);

    private static final double SHOOT_SPEED = 1;
    private static final double LOAD_SPEED = 0.9;

    private final double LiftSpeedRate = 0.07;

    public ShooterSubsystem() {
        rightShootMotor.setInverted(false);
        leftShootMotor.setInverted(true);
        loadMotor.setInverted(false);

        rightLiftMotor.setInverted(true);
        leftLiftMotor.setInverted(false);

        rightLiftMotor.setNeutralMode(NeutralModeValue.Brake);
        leftLiftMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void shoot() {
        rightShootMotor.set(SHOOT_SPEED);
        leftShootMotor.set(SHOOT_SPEED);
        // loadMotor.set(LoadSpeed);
    }

    public void stopShoot() {
        rightShootMotor.set(0);
        leftShootMotor.set(0);
        // loadMotor.set(0);
    }

    public void load() {
        loadMotor.set(LOAD_SPEED);
    }

    public void stopLoad() {
        loadMotor.set(0);
    }
    
    public void lift(double LiftSpeed) {
        rightLiftMotor.set(LiftSpeed * LiftSpeedRate);
        leftLiftMotor.set(LiftSpeed * LiftSpeedRate);
      }
    
    public void liftTo(double setpoint) {
        final double pidSpeed = liftPID.calculate(liftEncoder.getAbsolutePosition(), setpoint);
        rightLiftMotor.set(pidSpeed * LiftSpeedRate);
        leftLiftMotor.set(pidSpeed * LiftSpeedRate);
    }

    public void stopLift() {
        rightLiftMotor.set(0);
        leftLiftMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Shooter Position", liftEncoder.getAbsolutePosition());
        double mean = (this.leftShootMotor.getVelocity().getValue() + this.rightShootMotor.getVelocity().getValue()) / 2.0;
        double error = Math.abs(this.leftShootMotor.getVelocity().getValue() - this.rightShootMotor.getVelocity().getValue());
        SmartDashboard.putNumber("Shooter Speed", mean * Math.PI * Units.inchesToMeters(4));
        SmartDashboard.putNumber("Shooter Error", error * Math.PI * Units.inchesToMeters(4));
        SmartDashboard.putNumber("Loader Speed", loadMotor.getVelocity().getValue() * Math.PI * 0.0485 / 3);
    }
}
