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

public class ShooterSubsystem extends SubsystemBase{
    private static final double SHOOT_SPEED = 1.0;
    private static final double LIFT_COEFFICIENT = 0.07;

    private static final double SLOPE_TOGGLER_OFFSET = 351.23;
    private static final boolean SLOPE_TOGGLER_REVERSED = true;
    private static final double SLOPE_TOGGLER_GEAR_RATIO = 10.0/22.0;

    private static final double SLOPE_TOGGLER_MAX_LIMIT = 60.0;
    private static final double SLOPE_TOGGLER_MIN_LIMIT = 0.0;

    private final ModuleTalonFX rightShootMotor = new ModuleTalonFX(RobotCANPorts.RIGHT_SHOOTER.get());
    private final ModuleTalonFX leftShootMotor = new ModuleTalonFX(RobotCANPorts.LEFT_SHOOTER.get());

    private final ModuleTalonFX slopeTogglerMotor = new ModuleTalonFX(RobotCANPorts.SLOPE_TOGGLER.get());
    private final DutyCycleEncoder slopeTogglerEncoder = new DutyCycleEncoder(0);


    private final PIDController slopeTogglerPIDController = new PIDController(0.1, 0, 0);

    public double getSlopeTogglerDegrees() {
        return (Units.rotationsToDegrees(slopeTogglerEncoder.getAbsolutePosition()) - SLOPE_TOGGLER_OFFSET) * SLOPE_TOGGLER_GEAR_RATIO * (SLOPE_TOGGLER_REVERSED ? -1 : 1);
    }

    public ShooterSubsystem() {
        this.rightShootMotor.setInverted(false);
        this.leftShootMotor.setInverted(false);

        this.slopeTogglerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void shoot() {
        this.rightShootMotor.set(SHOOT_SPEED);
        this.leftShootMotor.set(SHOOT_SPEED);
    }

    public void stopShooters() {
        this.rightShootMotor.stopMotor();
        this.leftShootMotor.stopMotor();
    }

    public void toggleSlope(double activeDirection) {
        activeDirection /= Math.abs(activeDirection);
        final boolean atMaxLimit = this.getSlopeTogglerDegrees() >= SLOPE_TOGGLER_MAX_LIMIT;
        final boolean atMinLimit = this.getSlopeTogglerDegrees() <= SLOPE_TOGGLER_MIN_LIMIT;

        if (!((atMaxLimit && activeDirection < 0) || (atMinLimit && activeDirection > 0))) this.slopeTogglerMotor.set(activeDirection * LIFT_COEFFICIENT);
        else this.stopSlopeToggler();
    }

    public void toggleSlopeTo(double setpoint) {
        this.slopeTogglerMotor.set(this.slopeTogglerPIDController.calculate(this.getSlopeTogglerDegrees(), setpoint));
    }

    public void stopSlopeToggler() {
        this.slopeTogglerMotor.stopMotor();
    }

    public void stopAll() {
        this.stopShooters();
        this.stopSlopeToggler();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Elevation", this.getSlopeTogglerDegrees());
    }
}
