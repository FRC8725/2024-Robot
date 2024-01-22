// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotMap.SwervePort;

public class SwerveSubsystem extends SubsystemBase {
    private final mSwerveModule frontLeft = new mSwerveModule(SwervePort.kFrontLeftDriveMotor, SwervePort.kFrontLeftTurningMotor, DriveConstants.kFrontLeftDriveMotorReversed, DriveConstants.kFrontLeftTurningMotorReversed, SwervePort.kFrontLeftDriveAbsEncoder, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final mSwerveModule frontRight = new mSwerveModule(SwervePort.kFrontRightDriveMotor, SwervePort.kFrontRightTurningMotor, DriveConstants.kFrontRightDriveMotorReversed, DriveConstants.kFrontRightTurningEncoderReversed, SwervePort.kFrontRightDriveAbsEncoder, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final mSwerveModule backLeft = new mSwerveModule(SwervePort.kBackLeftDriveMotor, SwervePort.kBackLeftTurningMotor, DriveConstants.kBackLeftDriveMotorReversed, DriveConstants.kBackLeftTurningEncoderReversed, SwervePort.kBackLeftDriveAbsEncoder, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final mSwerveModule backRight = new mSwerveModule(SwervePort.kBackRightDriveMotor, SwervePort.kBackRightTurningMotor, DriveConstants.kBackRightDriveMotorReversed, DriveConstants.kBackRightTurningEncoderReversed, SwervePort.kBackRightDriveAbsEncoder, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final Field2d field = new Field2d();
    
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            Constants.DriveConstants.kDriveKinematics, getRotation2d(), this.getModulePositions()
    );

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveSubsystem() {
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                resetEncoders();
            } catch (Exception ignored) {}
        }).start();

        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getChassisSpeeds, 
            this::driveChassis, 
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(AutoConstants.kPathing_kP, AutoConstants.kPathing_kI, AutoConstants.kPathing_kD),
                        new PIDConstants(AutoConstants.kPathingTurning_kP, AutoConstants.kPathingTurning_kI, AutoConstants.kPathingTurning_kD),
                        AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                        0.3, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(true, true, 5, 5)), 
            () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, 
            this
        );
    }

    private ChassisSpeeds getChassisSpeeds() {
        return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(this.getModuleStates());
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void teleMove(Double xSpeed, Double ySpeed, Double rotation, boolean fieldOriented) {
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        rotation = turningLimiter.calculate(rotation) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    
        constructAndSetModuleStates(xSpeed, ySpeed, rotation, fieldOriented);
    }
    
    public void move(Double xSpeed, Double ySpeed, Double rotation, boolean fieldOriented) {
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        rotation = turningLimiter.calculate(rotation);
        constructAndSetModuleStates(xSpeed, ySpeed, rotation, fieldOriented);
    }
    
    private void constructAndSetModuleStates(Double xSpeed, Double ySpeed, Double rotation, boolean fieldOriented) {
        ChassisSpeeds chassisSpeeds = fieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, getRotation2d())
                : new ChassisSpeeds(ySpeed, xSpeed, rotation);
    
        driveChassis(chassisSpeeds);
    }

    private void driveChassis(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }
    
    public Command getResetGyroCommand() {
        return Commands.runOnce(this::zeroHeading, this);
    }

    public void lockModules() {
        frontLeft.lockModule();
        frontRight.lockModule();
        backLeft.lockModule();
        backRight.lockModule();
    }

    @Override
    public void periodic() {
        odometry.update(this.gyro.getRotation2d(), getModulePositions());

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putData(field);
        field.setRobotPose(getPose());
        backLeft.putDashboard();
        backRight.putDashboard();
        frontLeft.putDashboard();
        frontRight.putDashboard();
    }
}
