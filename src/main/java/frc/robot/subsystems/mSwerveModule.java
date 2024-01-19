package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.LazyTalonFX;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.mSwerveModuleConstants;

public class mSwerveModule {
    private final LazyTalonFX driveMotor;
    private final LazyTalonFX turningMotor;
    private final PIDController turningPIDController;
    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public mSwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
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

    public double getTurningPosition() {
        return turningMotor.getPositionAsRotation();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocityAsMPS(mSwerveModuleConstants.kWheelCircumference);
    }

    public double getAbsoluteEncoderRad() {
        double angle = getAbsPosition() / 180. * Math.PI;
        angle -= absoluteEncoderOffsetRad / 180 * Math.PI;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getAbsPosition() {
        var encoderPosSingal = absoluteEncoder.getAbsolutePosition();
        return Units.rotationsToDegrees(encoderPosSingal.getValue());
    }

    public void resetEncoders() {
        driveMotor.setRadPosition(0);
        turningMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        double driveVelocity =
            driveMotor.getVelocity().getValue() *  Constants.mSwerveModuleConstants.kDriveVelocityConversionFactor;

        return new SwerveModuleState(
            driveVelocity,
            new Rotation2d(getAbsoluteEncoderRad())
        );
    }

    public double getDriveMeters() {
        return driveMotor.getPositionAsRotation() * Constants.mSwerveModuleConstants.kDrivePositionConversionFactor;
    }

    public SwerveModulePosition getPosition() {
        double drivePosition = driveMotor.getPosition().getValue() * Constants.mSwerveModuleConstants.kDrivePositionConversionFactor;
    
        return new SwerveModulePosition(
            drivePosition,
            new Rotation2d(getAbsoluteEncoderRad())
        );
    }

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(MathUtil.applyDeadband(state.speedMetersPerSecond, 0.001));
        turningMotor.set(turningPIDController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        // SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
        putDashboard();
    }

    public void stop() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }

    public void lockModule() {
        switch (turningMotor.getDeviceID()) {
            case (RobotMap.kFrontLeftTurningMotorPort):
            case (RobotMap.kBackRightTurningMotorPort):
                turningMotor.set(turningPIDController.calculate(getAbsoluteEncoderRad(), -Math.PI / 4));
                break;
            case (RobotMap.kFrontRightTurningMotorPort):
            case (RobotMap.kBackLeftTurningMotorPort):
                turningMotor.set(turningPIDController.calculate(getAbsoluteEncoderRad(), Math.PI / 4));
                break;
        }
    }
    public void putDashboard() {
        SmartDashboard.putNumber("ABS angle " + absoluteEncoder.getDeviceID(), getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Postion" + absoluteEncoder.getDeviceID(), driveMotor.getPositionAsRotation());
        // SmartDashboard.putNumber("Abs Position " + absoluteEncoder.getDeviceID(), getAbsPosition());
    }
}//

