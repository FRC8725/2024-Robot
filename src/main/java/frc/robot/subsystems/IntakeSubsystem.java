package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ModuleTalonFX;
import frc.lib.helpers.IDashboardProvider;
import frc.robot.constants.RobotCANPorts;

public class IntakeSubsystem extends SubsystemBase implements IDashboardProvider {
    private static final double INTAKE_SPEED = 0.07;
    private static final double LIFT_COEFFICIENT = 0.07;
    private final ModuleTalonFX intakeMotor = new ModuleTalonFX(RobotCANPorts.INTAKE.get());
    private final CANSparkMax liftMotor = new CANSparkMax(RobotCANPorts.INTAKE_LIFTER.get(), MotorType.kBrushless);
    private final AbsoluteEncoder liftEncoder = this.liftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    private final PIDController liftPIDController = new PIDController(0.1, 0, 0);

    public IntakeSubsystem() {
        this.intakeMotor.setInverted(true);
        this.liftMotor.setIdleMode(IdleMode.kBrake);
        this.liftEncoder.setZeroOffset(0.907);
        this.liftEncoder.setInverted(true);
    }

    public void intake() {
        this.intakeMotor.set(INTAKE_SPEED);
    }

    public void stopIntake() {
        this.intakeMotor.stopMotor();
    }

    public void lift(double speed) {
        this.liftMotor.set(speed * LIFT_COEFFICIENT);
    }

    public void liftTo(double setpoint) {
        this.lift(this.liftPIDController.calculate(this.liftEncoder.getPosition(), setpoint));
    }

    public void stopLift() {
        this.liftMotor.stopMotor();
    }

    public void stopAll() {
        this.stopIntake();
        this.stopLift();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("IntakePos", this.liftEncoder.getPosition());
    }

    @Override
    public void putDashboardOnce() {
    }
}
