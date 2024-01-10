package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.LazyTalonFX;
import frc.robot.RobotMap;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
    private final LazyTalonFX driveMotor;
    private final LazyTalonFX turningMotor;
    private final PIDController turningPIDController;
    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderID);

        driveMotor = new LazyTalonFX(driveMotorId, SwerveModuleConstants.kDriveMotorGearRatio);
        turningMotor = new LazyTalonFX(turningMotorId, SwerveModuleConstants.kTurningMotorGearRatio);

        configDriveMotor(driveMotorReversed);
        configTurningMotor(turningMotorReversed);


        turningPIDController = new PIDController(SwerveModuleConstants.kPTurning, SwerveModuleConstants.kITurning, SwerveModuleConstants.kDTurning);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
        putDashboard();
    }

    private void configDriveMotor(boolean reversed) {
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.setCurrentLimit(true);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(reversed);
    }

    private void configTurningMotor(boolean reversed) {
        turningMotor.configFactoryDefault();
        turningMotor.setCurrentLimit(false);
        turningMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor.setInverted(reversed);
    }

    public double getTurningPosition() {
        return turningMotor.getPositionAsRad();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocityAsMPS(SwerveModuleConstants.kWheelCircumrerence);
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition() / 180. * Math.PI;
        angle -= absoluteEncoderOffsetRad / 180 * Math.PI;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setRadPosition(0);
        turningMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public double getDriveMeters() {
        return driveMotor.getPositionAsRad() * SwerveModuleConstants.kWheelDiameterMeters / 2;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveMeters(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(TalonFXControlMode.PercentOutput, turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
        putDashboard();
    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        turningMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void lockModule() {
        switch (turningMotor.getDeviceID()) {
            case (RobotMap.kFrontLeftTurningMotorPort):
            case (RobotMap.kBackRightTurningMotorPort):
                turningMotor.set(TalonFXControlMode.PercentOutput, turningPIDController.calculate(getTurningPosition(), -Math.PI / 4));
                break;
            case (RobotMap.kFrontRightTurningMotorPort):
            case (RobotMap.kBackLeftTurningMotorPort):
                turningMotor.set(TalonFXControlMode.PercentOutput, turningPIDController.calculate(getTurningPosition(), Math.PI / 4));
                break;
        }
    }

    public void putDashboard() {
        SmartDashboard.putNumber("ABS angle " + absoluteEncoder.getDeviceID(), getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Abs Position " + absoluteEncoder.getDeviceID(), absoluteEncoder.getAbsolutePosition());
    }
}

