package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ModuleTalonFX;
import frc.robot.RobotMap.ShooterPort;

public class ShooterSubsystem extends SubsystemBase {
    private final ModuleTalonFX rightShootMotor = new ModuleTalonFX(ShooterPort.kRightShootMotor);
    private final ModuleTalonFX leftShootMotor = new ModuleTalonFX(ShooterPort.kLeftShootMotor);
    private final ModuleTalonFX loadMotor = new ModuleTalonFX(ShooterPort.kloadMotor);

    private final ModuleTalonFX rightliftMotor = new ModuleTalonFX(ShooterPort.kRightLiftMotor);
    private final ModuleTalonFX leftliftMotor = new ModuleTalonFX(ShooterPort.kLeftLiftMotor);
    
    private final DutyCycleEncoder liftEncoder = new DutyCycleEncoder(1); // Shoot rotation without offset 0.98
    private final PIDController liftPID = new PIDController(0.1, 0, 0);

    private final double ShootSpeed = 1;
    private final double LoadSpeed = 0.9; 

    private final double LiftSpeedRate = 0.07;

    public ShooterSubsystem() {
        rightShootMotor.setInverted(false);
        leftShootMotor.setInverted(true);
        loadMotor.setInverted(false);

        rightliftMotor.setInverted(true);
        leftliftMotor.setInverted(false);

        rightliftMotor.setNeutralMode(NeutralModeValue.Brake);
        leftliftMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void shoot() {
        rightShootMotor.set(ShootSpeed);
        // leftShootMotor.set(ShootSpeed);
        // loadMotor.set(LoadSpeed);
    }

    public void stopShoot() {
        rightShootMotor.set(0);
        leftShootMotor.set(0);
        // loadMotor.set(0);
    }

    public void load() {
        loadMotor.set(LoadSpeed);
    }

    public void stopLoad() {
        loadMotor.set(0);
    }
    
    public void lift(double LiftSpeed) {
        rightliftMotor.set(LiftSpeed * LiftSpeedRate);
        leftliftMotor.set(LiftSpeed * LiftSpeedRate);
      }
    
    public void liftTo(double setpoint) {
        final double pidSpeed = liftPID.calculate(liftEncoder.getAbsolutePosition(), setpoint);
        rightliftMotor.set(pidSpeed * LiftSpeedRate);
        leftliftMotor.set(pidSpeed * LiftSpeedRate);
    }

    public void stopLift() {
        rightliftMotor.set(0);
        leftliftMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Shooter Position", liftEncoder.getAbsolutePosition());

        SmartDashboard.putNumber("Shooter Speed", rightShootMotor.getVelocity().getValue() * Math.PI*Units.inchesToMeters(4) / 3);
        SmartDashboard.putNumber("Loader Speed", loadMotor.getVelocity().getValue() * Math.PI*0.0485 / 3);
    }
}
