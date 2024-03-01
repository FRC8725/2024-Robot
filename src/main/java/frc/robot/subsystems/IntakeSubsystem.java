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
import frc.lib.helpers.UnitTypes;
import frc.lib.math.MathHelper;
import frc.robot.constants.RobotPorts;

public class IntakeSubsystem extends SubsystemBase implements IDashboardProvider {
    @OutputUnit(UnitTypes.DEGREES)
    public static final double LIFTER_MAX_SETPOINT = 224.5;
    @OutputUnit(UnitTypes.DEGREES)
    public static final double LIFTER_MAX_BRAKE_SETPOINT = 180.0;
    @OutputUnit(UnitTypes.DEGREES)
    public static final double LIFTER_MIN_SETPOINT = 62.0;
    @OutputUnit(UnitTypes.DEGREES)
    public static final double LIFTER_AMP_SETPOINT = 154.0; // Setpoint for scoring AMP
    @OutputUnit(UnitTypes.DEGREES)
    private static final double DEFAULT_LIFTER_THRESHOLD = 5.0; // Determine whether the lifter reaches the position

    @OutputUnit(UnitTypes.PERCENTAGES)
    private static final double INTAKE_SPEED = 0.4;
    @OutputUnit(UnitTypes.PERCENTAGES)
    private static final double RELEASE_SPEED = 1.0;
    @OutputUnit(UnitTypes.PERCENTAGES)
    private static final double AMP_SPEED = 0.75;
    @OutputUnit(UnitTypes.PERCENTAGES)
    private static final double FINE_TUNE_NOTE_SPEED = 0.04;

    @OutputUnit(UnitTypes.DEGREES)
    private static final double LIFTER_ZERO_OFFSET = 172.0;
    @OutputUnit(UnitTypes.DEGREES)
    private static final double LIFTER_FORCE_OFFSET = 0.0;
    private static final boolean LIFTER_REVERSED = true;
    private static final double LIFTER_GEAR_RATIO = 18.0 / 22.0;

    private final ModuleTalonFX rightIntakeMotor = new ModuleTalonFX(RobotPorts.CAN.RIGHT_INTAKE.get());
    private final ModuleTalonFX leftIntakeMotor = new ModuleTalonFX(RobotPorts.CAN.LEFT_INTAKE.get());

    private final ModuleTalonFX rightLiftMotor = new ModuleTalonFX(RobotPorts.CAN.RIGHT_INTAKE_LIFTER.get());
    private final ModuleTalonFX leftLiftMotor = new ModuleTalonFX(RobotPorts.CAN.LEFT_INTAKE_LIFTER.get());

    private final DutyCycleEncoder liftEncoder = new DutyCycleEncoder(RobotPorts.DIO.LIFTER_ENCODER.get());
    private final PIDController liftPIDController = new PIDController(0.006, 0.0, 0.0);
    private final PIDController testPIDController = new PIDController(0.01, 0.0, 0.0);

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
        double position = Units.rotationsToDegrees(this.liftEncoder.getAbsolutePosition());
        position = (position + LIFTER_ZERO_OFFSET) % 360;
        position = position * LIFTER_GEAR_RATIO * (LIFTER_REVERSED ? 1.0 : -1.0);
        return position + LIFTER_FORCE_OFFSET;
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

    public void releaseAMP() {
        this.executeIntake(-AMP_SPEED);
    }

    public void fineTuneNote() {
        this.executeIntake(FINE_TUNE_NOTE_SPEED);
    }

    private void liftIntake(double percentage) {
        this.rightLiftMotor.set(percentage);
        this.leftLiftMotor.set(percentage);
    }

    public void liftTo(double setpoint) {
        this.liftIntake(this.liftPIDController.calculate(this.getLifterPosition(), setpoint));
    }

    public void liftToMax() {
        double output = this.testPIDController.calculate(this.getLifterPosition(), 175.0) + 0.2;
        if (output < 0.2) output = 0.2;
        if (this.getLifterPosition() > LIFTER_MAX_BRAKE_SETPOINT) {
            output = this.liftPIDController.calculate(this.getLifterPosition(), LIFTER_MAX_SETPOINT);
            output = MathHelper.applyMax(output, 0.2);
        }
            
        this.liftIntake(output);

        if (!this.isLifterAtMax()) this.fineTuneNote();
        else this.stopIntake();
    }

    public void liftToMin() {
        this.liftTo(LIFTER_MIN_SETPOINT);
    }

    public void liftToAMP() {
        this.liftTo(LIFTER_AMP_SETPOINT);

        if (!this.isLifterAt(LIFTER_AMP_SETPOINT, DEFAULT_LIFTER_THRESHOLD)) this.fineTuneNote();
        else this.stopIntake();
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
