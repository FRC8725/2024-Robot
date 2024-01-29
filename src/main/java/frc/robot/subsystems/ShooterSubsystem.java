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
    private static final double LIFT_COEFFICIENT = 0.07;

    private final ModuleTalonFX rightShootMotor = new ModuleTalonFX(RobotCANPorts.RIGHT_SHOOTER.get());
    private final ModuleTalonFX leftShootMotor = new ModuleTalonFX(RobotCANPorts.LEFT_SHOOTER.get());

    private final ModuleTalonFX angleTogglerMotor = new ModuleTalonFX(RobotCANPorts.ANGLETOGGLER.get());
    private final DutyCycleEncoder angleTogglerEncoder = new DutyCycleEncoder(0);
    private final PIDController angleTogglerPID = new PIDController(0.1, 0, 0);

    public ShooterSubsystem() {
        this.rightShootMotor.setInverted(false);
        this.leftShootMotor.setInverted(true);

        this.angleTogglerMotor.setNeutralMode(NeutralModeValue.Brake);
        this.angleTogglerEncoder.setPositionOffset(0);
    }

    public void shoot() {
        this.rightShootMotor.set(SHOOT_SPEED);
        this.leftShootMotor.set(SHOOT_SPEED);
    }

    public void stopShooters() {
        this.rightShootMotor.stopMotor();
        this.leftShootMotor.stopMotor();
    }

    public void toggleAngle(double activeDirection) {
        activeDirection /= Math.abs(activeDirection);
        this.angleTogglerMotor.set(activeDirection * LIFT_COEFFICIENT);
    }

    public void toggleAngleTo(double setpoint) {
        this.angleTogglerMotor.set(this.angleTogglerPID.calculate(this.angleTogglerEncoder.getAbsolutePosition(), setpoint));
    }

    public void stopAngleToggler() {
        this.angleTogglerMotor.stopMotor();
    }

    public void stopAll() {
        this.stopShooters();
        this.stopAngleToggler();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Shooter Position", angleTogglerEncoder.getAbsolutePosition());
    }

    @Override
    public void putDashboardOnce() {
    }
}
