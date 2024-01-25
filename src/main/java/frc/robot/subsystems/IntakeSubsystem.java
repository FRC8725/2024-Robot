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
import frc.robot.RobotMap.IntakePort;

public class IntakeSubsystem extends SubsystemBase {
    private final ModuleTalonFX intakeMotor = new ModuleTalonFX(IntakePort.kIntakeMotor);
    private final CANSparkMax liftMotor = new CANSparkMax(IntakePort.kLiftMotor, MotorType.kBrushless);
    private final AbsoluteEncoder liftEncoder = liftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    private final PIDController liftPID = new PIDController(0.1, 0, 0);

    private final double IntakeSpeed = 0.07;
    private final double LiftSpeedRate = 0.07;

    public IntakeSubsystem() {
        intakeMotor.setInverted(true);

        liftMotor.setIdleMode(IdleMode.kBrake);
        liftEncoder.setZeroOffset(0.907);
        liftEncoder.setInverted(true);
    }

    public void intake() {
        intakeMotor.set(IntakeSpeed);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void lift(double LiftSpeed) {
        liftMotor.set(LiftSpeed * LiftSpeedRate);
    }

    public void liftTo(double setpoint) {
        final double pidSpeed = liftPID.calculate(liftEncoder.getPosition(), setpoint);
        liftMotor.set(pidSpeed * LiftSpeedRate);
    }

    public void stopLift() {
        liftMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake Position", liftEncoder.getPosition());
    }
}
