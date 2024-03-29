package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ModuleTalonFX;
import frc.lib.helpers.IDashboardProvider;
import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.TidiedUp;
import frc.lib.helpers.UnitTypes;
import frc.lib.math.MathHelper;
import frc.robot.constants.RobotPorts;

public class ShooterSubsystem extends SubsystemBase implements IDashboardProvider {
    @OutputUnit(UnitTypes.PERCENTAGES)
    private static final double SHOOT_SPEED = 1.0; // 0.9
    @OutputUnit(UnitTypes.PERCENTAGES)
    private static final double SHOOT_THRESHOLD = 0.23;
    @OutputUnit(UnitTypes.PERCENTAGES)
    private static final double SOURCE_COLLECT_SPEED = 0.2;

    private final ModuleTalonFX rightShootMotor = new ModuleTalonFX(RobotPorts.CAN.LEFT_SHOOTER.get());
    private final ModuleTalonFX leftShootMotor = new ModuleTalonFX(RobotPorts.CAN.RIGHT_SHOOTER.get());

    private final PIDController shooterPIDController = new PIDController(0.1, 3.0, 0.0); // TODO re-tune the PID

    public ShooterSubsystem() {
        this.rightShootMotor.setInverted(false);
        this.leftShootMotor.setInverted(false);

        this.registerDashboard();
    }

    public double getAverageShooterSpeed() {
        double mean = MathHelper.getMean(this.getLeftShooterSpeed(), this.getRightShooterSpeed());
        return mean / 90.0; // TODO I don't understand the division performed here
    }

    // TODO specify the unit, I don't understand
    private double getRightShooterSpeed() {
        return this.rightShootMotor.getVelocity().getValue();
    }

    // TODO specify the unit, I don't understand
    private double getLeftShooterSpeed() {
        return this.leftShootMotor.getVelocity().getValue();
    }

    public boolean canShoot() {
        return MathHelper.isWithinRange(this.getAverageShooterSpeed(), SHOOT_SPEED, SHOOT_THRESHOLD);
    }

    public void executeShooter() {
        final double currentSpeed = this.getAverageShooterSpeed();
        final double output = this.shooterPIDController.calculate(currentSpeed, SHOOT_SPEED);

        // TODO turn this to setVoltage
        this.rightShootMotor.set(output);
        this.leftShootMotor.set(output);

        // SmartDashboard.putNumber("Shooter Speed", currentSpeed);
        // SmartDashboard.putNumber("Shooter Output", output);
    }

    public void collectSource() {
        this.rightShootMotor.set(-SOURCE_COLLECT_SPEED);
        this.leftShootMotor.set(-SOURCE_COLLECT_SPEED);
    }

    public void stopAll() {
        this.rightShootMotor.stopMotor();
        this.leftShootMotor.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Shooter Speed", this.getAverageShooterSpeed());
        SmartDashboard.putBoolean("Can Shoot", this.canShoot());
    }

    @Override
    public void putDashboardOnce() {
    }
}
