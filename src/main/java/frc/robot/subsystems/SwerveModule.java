package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.LazyTalonFX;
import frc.lib.helpers.IDashboardProvider;
import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.UnitTypes;
import frc.robot.RobotMap.SwervePort;

public class SwerveModule implements IDashboardProvider {
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    private static final double DRIVING_GEAR_RATIO = 7.0 / 57.0;
    private static final double STEERING_GEAR_RATIO = 7.0 / 150.0;
    private final LazyTalonFX driveMotor;
    private final LazyTalonFX turningMotor;
    private final CANcoder absoluteEncoder;
    private final double absEncoderOffset;
    private final PIDController turningPIDController = new PIDController(0.45, 0.0, 0.0);
    private final String name;

    public SwerveModule(String name, int driveMotorId, int turningMotorId, boolean driveMotorInverted, boolean turningMotorInverted, int absoluteEncoderID, double absEncoderOffset) {
        this.name = name;
        this.absEncoderOffset = absEncoderOffset;
        this.absoluteEncoder = new CANcoder(absoluteEncoderID);

        this.driveMotor = new LazyTalonFX(driveMotorId, DRIVING_GEAR_RATIO);
        this.turningMotor = new LazyTalonFX(turningMotorId, STEERING_GEAR_RATIO);
        this.configDriveMotor(driveMotorInverted);
        this.configTurningMotor(turningMotorInverted);

        this.turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.resetRelativeEncoders();
        this.registerDashboard();
    }

    private void configDriveMotor(boolean inverted) {
        this.driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        this.driveMotor.setCurrentLimit(true);
        this.driveMotor.setNeutralMode(NeutralModeValue.Brake);
        this.driveMotor.setInverted(inverted);
    }

    private void configTurningMotor(boolean inverted) {
        this.turningMotor.getConfigurator().apply(new TalonFXConfiguration());
        this.turningMotor.setCurrentLimit(false);
        this.turningMotor.setNeutralMode(NeutralModeValue.Brake);
        this.turningMotor.setInverted(inverted);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.getDrivePosition(), new Rotation2d(this.getAbsTurningPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getDriveVelocity(), new Rotation2d(this.getAbsTurningPosition()));
    }

    @OutputUnit(UnitTypes.METERS)
    private double getDrivePosition() {
        double rawPosition = this.driveMotor.getPosition().getValue();
        return rawPosition * DRIVING_GEAR_RATIO * 2.0 * Math.PI * WHEEL_RADIUS;
    }

    @OutputUnit(UnitTypes.METERS_PER_SECOND)
    private double getDriveVelocity() {
        double rawVelocity = this.driveMotor.getVelocity().getValue();
        return rawVelocity * DRIVING_GEAR_RATIO * 2.0 * Math.PI * WHEEL_RADIUS;
    }

    @OutputUnit(UnitTypes.RADIANS)
    private double getAbsTurningPosition() { // value in [-Math.PI, Math.PI)
        double position = this.absoluteEncoder.getAbsolutePosition().getValue();
        position = Units.rotationsToRadians(position) - this.absEncoderOffset;
        return Math.IEEEremainder(position, 2.0 * Math.PI);
    }

    public void resetRelativeEncoders() {
        this.driveMotor.setRadPosition(0);
        this.turningMotor.setRadPosition(this.getAbsTurningPosition());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }

        desiredState = SwerveModuleState.optimize(desiredState, this.getState().angle);
        this.driveMotor.set(desiredState.speedMetersPerSecond);
        final double desiredAngle = desiredState.angle.getRadians();
        final double currentAngle = this.getState().angle.getRadians();
        this.turningMotor.set(this.turningPIDController.calculate(currentAngle, desiredAngle));
        // SmartDashboard.putString(this.name + " desiredState", desiredState.toString());
    }

    public void stop() {
        this.driveMotor.stopMotor();
        this.turningMotor.stopMotor();
    }

    public void lockModule() {
        switch (this.turningMotor.getDeviceID()) {
            case (SwervePort.kFrontLeftTurningMotor):
            case (SwervePort.kBackRightTurningMotor):
                this.turningMotor.set(this.turningPIDController.calculate(this.getAbsTurningPosition(), Math.PI / 4));
                break;
            case (SwervePort.kFrontRightTurningMotor):
            case (SwervePort.kBackLeftTurningMotor):
                this.turningMotor.set(this.turningPIDController.calculate(this.getAbsTurningPosition(), -Math.PI / 4));
                break;
        }
    }

    public void putDashboard() {
        SmartDashboard.putNumber(this.name + " DrivePos", this.getPosition().distanceMeters);
        SmartDashboard.putNumber(this.name + " AbsTurnPos", this.getPosition().angle.getDegrees());
    }

    @Override
    public void putDashboardOnce() {
    }
}
