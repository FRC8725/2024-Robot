package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ModuleTalonFX;
import frc.lib.math.TrajectoryEstimator;
import frc.robot.constants.RobotCANPorts;

public class ShooterSubsystem extends SubsystemBase{
    private static final double SHOOT_SPEED = 1.0;
    private static final double LIFT_COEFFICIENT = 0.07;

    private static final double SLOPE_TOGGLER_ZERO_OFFSET = 233.52;//;
    private static final boolean SLOPE_TOGGLER_REVERSED = true;
    private static final double SLOPE_TOGGLER_GEAR_RATIO = 10.0/22.0;

    private static final double SLOPE_TOGGLER_MAX_LIMIT = 50.0;
    private static final double SLOPE_TOGGLER_MIN_LIMIT = 2.0;

    private final ModuleTalonFX rightShootMotor = new ModuleTalonFX(RobotCANPorts.RIGHT_SHOOTER.get());
    private final ModuleTalonFX leftShootMotor = new ModuleTalonFX(RobotCANPorts.LEFT_SHOOTER.get());

    private final ModuleTalonFX slopeTogglerMotor = new ModuleTalonFX(RobotCANPorts.SLOPE_TOGGLER.get());
    private final DutyCycleEncoder slopeTogglerEncoder = new DutyCycleEncoder(0);


    private final PIDController slopeTogglerPIDController = new PIDController(0.06, 0, 0);
    private final PIDController shooterPIDController = new PIDController(0.1, 1, 0);

    public double getSlopeTogglerDegrees() {
        return (Units.rotationsToDegrees(slopeTogglerEncoder.getAbsolutePosition()) - SLOPE_TOGGLER_ZERO_OFFSET) * (SLOPE_TOGGLER_REVERSED ? -1 : 1) * SLOPE_TOGGLER_GEAR_RATIO;
    }

    public ShooterSubsystem() {
        this.rightShootMotor.setInverted(false);
        this.leftShootMotor.setInverted(false);

        this.slopeTogglerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private double getShooterSpeed() {
        return ((this.rightShootMotor.getVelocity().getValue() + this.leftShootMotor.getVelocity().getValue()) / 2) / 90;
    }

    public void shoot() {
        final double currentSpeed = getShooterSpeed();
        final double output = shooterPIDController.calculate(currentSpeed, SHOOT_SPEED); 
        this.rightShootMotor.set(output);
        this.leftShootMotor.set(output);
        
        // SmartDashboard.putNumber("Shooter Speed", currentSpeed);
        // SmartDashboard.putNumber("Shooter Output", output);
    }

    public void stopShooters() {
        this.rightShootMotor.stopMotor();
        this.leftShootMotor.stopMotor();
    }

    public void toggleSlope(double activeDirection) {
        activeDirection /= Math.abs(activeDirection);
        final boolean atMaxLimit = this.getSlopeTogglerDegrees() >= SLOPE_TOGGLER_MAX_LIMIT;
        final boolean atMinLimit = this.getSlopeTogglerDegrees() <= SLOPE_TOGGLER_MIN_LIMIT;

        // this.slopeTogglerMotor.set(activeDirection * LIFT_COEFFICIENT);

        if (!((atMaxLimit && activeDirection < 0) || (atMinLimit && activeDirection > 0))) this.slopeTogglerMotor.set(activeDirection * LIFT_COEFFICIENT);
        else this.stopSlopeToggler();
    }

    public void toggleSlopeWithDistance(double distance) {
        SmartDashboard.putNumber("Speaker dis", distance);
        this.toggleSlopeTo(TrajectoryEstimator.getAngleOfElevation(distance));
    }

    public void toggleSlopeTo(double setpoint) {
        final double output = this.slopeTogglerPIDController.calculate(this.getSlopeTogglerDegrees(), setpoint) * (SLOPE_TOGGLER_REVERSED ? -1 : 1);
        SmartDashboard.putNumber("slope setpoint", setpoint);
        SmartDashboard.putNumber("slope output", output);
        this.slopeTogglerMotor.set(output);
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
        SmartDashboard.putNumber("Shooter Speed", this.getShooterSpeed());
    }
}
