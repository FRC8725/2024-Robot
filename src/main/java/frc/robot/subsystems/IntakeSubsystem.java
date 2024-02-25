package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ModuleTalonFX;
import frc.lib.helpers.IDashboardProvider;
import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.TidiedUp;
import frc.lib.helpers.UnitTypes;
import frc.lib.math.MathHelper;
import frc.robot.constants.RobotCANPorts;
import org.apache.commons.math3.util.FastMath;

@TidiedUp
public class IntakeSubsystem extends SubsystemBase implements IDashboardProvider {
    @OutputUnit(UnitTypes.DEGREES)
    public static final double LIFTER_MAX_SETPOINT = 170.0;
    @OutputUnit(UnitTypes.DEGREES)
    public static final double LIFTER_MIN_SETPOINT = 11.0;
    @OutputUnit(UnitTypes.DEGREES)
    public static final double LIFTER_AMP_SETPOINT = 103.86; // Setpoint for scoring AMP
    @OutputUnit(UnitTypes.DEGREES)
    private static final double DEFAULT_LIFTER_THRESHOLD = 0.1; // Determine whether the lifter reaches the position

    @OutputUnit(UnitTypes.PERCENTAGES)
    private static final double INTAKE_SPEED = 0.4;
    @OutputUnit(UnitTypes.PERCENTAGES)
    private static final double RELEASE_SPEED = 1.0;
    @OutputUnit(UnitTypes.PERCENTAGES)
    private static final double FINE_TUNE_NOTE_SPEED = 0.03;
    @OutputUnit(UnitTypes.PERCENTAGES)
    private static final double LIFT_PID_COEFFICIENT = 0.1;

    @OutputUnit(UnitTypes.DEGREES)
    private static final double LIFTER_ZERO_OFFSET = 172.1;
    private static final boolean LIFTER_REVERSED = false;
    private static final double LIFTER_GEAR_RATIO = 18.0 / 22.0;

    private final ModuleTalonFX rightIntakeMotor = new ModuleTalonFX(RobotCANPorts.RIGHT_INTAKE.get());
    private final ModuleTalonFX leftIntakeMotor = new ModuleTalonFX(RobotCANPorts.LEFT_INTAKE.get());

    private final ModuleTalonFX rightLiftMotor = new ModuleTalonFX(RobotCANPorts.RIGHT_INTAKE_LIFTER.get());
    private final ModuleTalonFX leftLiftMotor = new ModuleTalonFX(RobotCANPorts.LEFT_INTAKE_LIFTER.get());

    private final DutyCycleEncoder liftEncoder = new DutyCycleEncoder(1);
    private final PIDController liftPIDController = new PIDController(0.04, 0.0, 0.0); // TODO turn this into ProfiledPID

    public IntakeSubsystem() {
        this.rightIntakeMotor.setInverted(true);
        this.leftIntakeMotor.setInverted(true);

        this.leftIntakeMotor.setNeutralMode(NeutralModeValue.Coast);
        this.rightLiftMotor.setNeutralMode(NeutralModeValue.Coast);

        this.rightLiftMotor.setInverted(true);
        this.leftLiftMotor.setInverted(false);

        this.rightLiftMotor.setNeutralMode(NeutralModeValue.Brake);
        this.leftLiftMotor.setNeutralMode(NeutralModeValue.Brake);

        this.registerDashboard();
    }

    @OutputUnit(UnitTypes.DEGREES)
    private double getLifterPosition() {
        double position = Units.degreesToRadians(this.liftEncoder.getAbsolutePosition());
        position = (position + LIFTER_ZERO_OFFSET) % 360;
        return position * LIFTER_GEAR_RATIO * (LIFTER_REVERSED ? 1.0 : -1.0);
    }

    private void executeIntake(double percentage) {
        this.rightIntakeMotor.set(percentage);
        this.leftIntakeMotor.set(percentage);
    }

    public void executeIntake() {
        this.executeIntake(INTAKE_SPEED);
    }

    public void releaseIntake() {
        this.executeIntake(-RELEASE_SPEED);
    }

    public void fineTuneNote() {
        this.executeIntake(FINE_TUNE_NOTE_SPEED);
    }

    private void liftIntakePID(double controllerOutput) {
        this.rightLiftMotor.setVoltage(controllerOutput);
        this.leftLiftMotor.setVoltage(controllerOutput);
    }

    private void liftIntake(double percentage) {
        this.rightLiftMotor.set(percentage);
        this.leftLiftMotor.set(percentage);
    }

    public void liftTo(double setpoint) {
        // TODO remove the coefficient
        final double output = this.liftPIDController.calculate(this.getLifterPosition(), setpoint) * LIFT_PID_COEFFICIENT;
        // SmartDashboard.putNumber("output", output);
        this.liftIntake(output); // TODO re-tune the PID controller, use liftIntakePID instead
    }

    public void liftToMin() {
        this.liftTo(LIFTER_MIN_SETPOINT);
    }

    public void liftToMax() {
        this.liftTo(LIFTER_MAX_SETPOINT);
        this.fineTuneNote();
    }

    public boolean isLifterAt(double setpoint, double threshold) {
        return MathHelper.isWithinRange(setpoint, this.getLifterPosition(), threshold);
    }

    public boolean isLifterAtMin() {
        return this.isLifterAt(LIFTER_MIN_SETPOINT, DEFAULT_LIFTER_THRESHOLD);
    }

    public boolean isLifterAtMax() {
        return this.isLifterAt(LIFTER_MAX_SETPOINT, DEFAULT_LIFTER_THRESHOLD);
    }

    public void stopIntake() {
        this.rightIntakeMotor.stopMotor();
        this.leftIntakeMotor.stopMotor();
    }

    public void stopLifters() {
        this.rightLiftMotor.stopMotor();
        this.leftLiftMotor.stopMotor();
    }

    public void stopAll() {
        this.stopIntake();
        this.stopLifters();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("IntakePos", this.getLifterPosition());
    }

    @Override
    public void putDashboardOnce() {
    }
}
