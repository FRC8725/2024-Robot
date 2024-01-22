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
import frc.robot.RobotMap.SwervePort;

import frc.robot.Constants.mSwerveModuleConstants;

public class mSwerveModule {
    private final LazyTalonFX driveMotor;
    private final LazyTalonFX turningMotor;
    private final PIDController turningPIDController;
    private final CANcoder absoluteEncoder;

    private final double absoluteEncoderOffsetRad;

    public mSwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        absoluteEncoder = new CANcoder(absoluteEncoderID);

        driveMotor = new LazyTalonFX(driveMotorId, mSwerveModuleConstants.kDriveMotorGearRatio);
        turningMotor = new LazyTalonFX(turningMotorId, mSwerveModuleConstants.kTurningMotorGearRatio);
        configDriveMotor(driveMotorReversed);
        configTurningMotor(turningMotorReversed);

        turningPIDController = new PIDController(mSwerveModuleConstants.kTurning_kP, mSwerveModuleConstants.kTurning_kI, mSwerveModuleConstants.kTurning_kD);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
        putDashboard();
    }

    private void configDriveMotor(boolean reversed) {
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.setCurrentLimit(true);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.setInverted(reversed);

    }

    private void configTurningMotor(boolean reversed) {
        turningMotor.getConfigurator().apply(new TalonFXConfiguration());
        turningMotor.setCurrentLimit(false);
        turningMotor.setNeutralMode(NeutralModeValue.Brake);
        turningMotor.setInverted(reversed);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePosition(),
            new Rotation2d(this.getAbsoluteEncoderRad())
        );
    }


    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            new Rotation2d(this.getAbsoluteEncoderRad())
        );
    }

    public double getDrivePosition() {
        return this.driveMotor.getDrivePositionAsMeter(mSwerveModuleConstants.kWheelCircumference);// * (this.driveMotorReversed ? -1 : 1);
        // return driveMotor.getPositionAsRotation() * Constants.mSwerveModuleConstants.kDrivePositionConversionFactor;
    }

    public double getDriveVelocity() {
        return this.driveMotor.getVelocityAsMPS(mSwerveModuleConstants.kWheelCircumference);// * (this.driveMotorReversed ? -1 : 1);
        // return driveMotor.getVelocityAsMPS(mSwerveModuleConstants.kWheelCircumference);
    }

    public double getAbsoluteEncoderRad() {
        double angle = Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition().getValue());
        // angle = -angle + 2*Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle;// * (absoluteEncoderReversed ? 1.0 : -1.0);
    }

    public void resetEncoders() {
        driveMotor.setRadPosition(0);
        turningMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond);
        turningMotor.set(-turningPIDController.calculate(this.getState().angle.getRadians(), state.angle.getRadians()));
        // SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
        putDashboard();
    }

    public void stop() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }

    public void lockModule() {
        switch (turningMotor.getDeviceID()) {
            case (SwervePort.kFrontLeftTurningMotor):
            case (SwervePort.kBackRightTurningMotor):
                turningMotor.set(-turningPIDController.calculate(getAbsoluteEncoderRad(), Math.PI / 4));
                break;
            case (SwervePort.kFrontRightTurningMotor):
            case (SwervePort.kBackLeftTurningMotor):
                turningMotor.set(-turningPIDController.calculate(getAbsoluteEncoderRad(), -Math.PI / 4));
                break;
        }
    }
    public void putDashboard() {
        SmartDashboard.putNumber("ABS angle " + absoluteEncoder.getDeviceID(), this.getState().angle.getRadians());
        SmartDashboard.putNumber("DriveMeter" + absoluteEncoder.getDeviceID(), driveMotor.getDrivePositionAsMeter(mSwerveModuleConstants.kWheelCircumference));
        // SmartDashboard.putNumber("Postion" + absoluteEncoder.getDeviceID(), driveMotor.getPositionAsRad());
        // SmartDashboard.putNumber("Abs Position " + absoluteEncoder.getDeviceID(), getAbsPosition());
    }
}//

