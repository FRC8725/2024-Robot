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

public class IntakeSubsystem extends SubsystemBase implements IDashboardProvider {
    private static final double INTAKE_SPEED = 0.4;
    private static final double RELEASE_SPEED = -1;
    private static final double LIFT_COEFFICIENT = 0.25;
    private final ModuleTalonFX rightIntakeMotor = new ModuleTalonFX(RobotCANPorts.RIGHT_INTAKE.get());
    private final ModuleTalonFX leftIntakeMotor = new ModuleTalonFX(RobotCANPorts.LEFT_INTAKE.get());

    private final ModuleTalonFX rightLiftMotor = new ModuleTalonFX(RobotCANPorts.RIGHT_INTAKE_LIFTER.get());
    private final ModuleTalonFX leftLiftMotor = new ModuleTalonFX(RobotCANPorts.LEFT_INTAKE_LIFTER.get());

    private final DutyCycleEncoder liftEncoder = new DutyCycleEncoder(2);
    private final PIDController liftPIDController = new PIDController(0.1, 0, 0);

    public IntakeSubsystem() {
        this.rightIntakeMotor.setInverted(true);
        this.leftIntakeMotor.setInverted(true);

        this.leftIntakeMotor.setNeutralMode(NeutralModeValue.Coast);
        this.rightLiftMotor.setNeutralMode(NeutralModeValue.Coast);

        this.rightLiftMotor.setInverted(true);
        this.leftLiftMotor.setInverted(false);
        
        this.rightLiftMotor.setNeutralMode(NeutralModeValue.Brake);
        this.leftLiftMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void intake() {
        this.rightIntakeMotor.set(INTAKE_SPEED);
        this.leftIntakeMotor.set(INTAKE_SPEED);
    }

    public void release() {
        this.rightIntakeMotor.set(RELEASE_SPEED);
        this.leftIntakeMotor.set(RELEASE_SPEED);
    }

    public void stopIntake() {
        this.rightIntakeMotor.stopMotor();
        this.leftIntakeMotor.stopMotor();
    }

    public void lift(double activeDirection) {
        activeDirection /= Math.abs(activeDirection);
        this.rightLiftMotor.set(activeDirection * LIFT_COEFFICIENT);
        this.leftLiftMotor.set(activeDirection * LIFT_COEFFICIENT);
    }

    public void liftTo(double setpoint) {
        final double output = this.liftPIDController.calculate(this.liftEncoder.getAbsolutePosition(), setpoint);
        this.rightLiftMotor.set(output);
        this.leftLiftMotor.set(output);
    }

    public void stopLift() {
        this.rightLiftMotor.stopMotor();
        this.leftLiftMotor.stopMotor();
    }

    public void stopAll() {
        this.stopIntake();
        this.stopLift();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("IntakePos", this.liftEncoder.getAbsolutePosition());
    }

    @Override
    public void putDashboardOnce() {
    }
}
