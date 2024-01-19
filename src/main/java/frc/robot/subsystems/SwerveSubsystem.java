// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MusicTone;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotMap;

public class SwerveSubsystem extends SubsystemBase {
    private final mSwerveModule frontLeft = new mSwerveModule(RobotMap.kFrontLeftDriveMotorPort, RobotMap.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftDriveMotorReversed, DriveConstants.kFrontLeftTurningMotorReversed, RobotMap.kFrontLeftDriveAbsoluteEncoderPort, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final mSwerveModule frontRight = new mSwerveModule(RobotMap.kFrontRightDriveMotorPort, RobotMap.kFrontRightTurningMotorPort, DriveConstants.kFrontRightDriveEncoderReversed, DriveConstants.kFrontRightTurningEncoderReversed, RobotMap.kFrontRightDriveAbsoluteEncoderPort, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final mSwerveModule backLeft = new mSwerveModule(RobotMap.kBackLeftDriveMotorPort, RobotMap.kBackLeftTurningMotorPort, DriveConstants.kBackLeftDriveEncoderReversed, DriveConstants.kBackLeftTurningEncoderReversed, RobotMap.kBackLeftDriveAbsoluteEncoderPort, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final mSwerveModule backRight = new mSwerveModule(RobotMap.kBackRightDriveMotorPort, RobotMap.kBackRightTurningMotorPort, DriveConstants.kBackRightDriveEncoderReversed, DriveConstants.kBackRightTurningEncoderReversed, RobotMap.kBackRightDriveAbsoluteEncoderPort, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final Field2d field = new Field2d();
    private final SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        this.getRotation2d(),
        getModulePositions()
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
                        new PIDConstants(AutoConstants.kPathingX_kP, 0.0, 0.0),
                        new PIDConstants(AutoConstants.kPathingTurning_kP, 0.0, 0.0),
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ), 
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
        double velocityx = gyro.getVelocityX();
        double velocityy = gyro.getVelocityY();

        double omegaRadPerSec = Units.degreesToRadians(gyro.getRate());
        
        return new ChassisSpeeds(velocityx, velocityy, omegaRadPerSec);
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
        return mOdometry.getPoseMeters();
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
        mOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
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
        mOdometry.update(getRotation2d(), getModulePositions());
        // SwerveEstimator.update(getRotation2d(), getModulePositions());
        // var gloabalPose = vision.getEstimatedGlobalPose();
        // if (vision.hasTarget()) SwerveEstimator.addVisionMeasurement(gloabalPose.get().getFirst(), gloabalPose.get().getSecond());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        // SmartDashboard.putNumber("Robot Pitch", getPitch());
        SmartDashboard.putData(field);
        field.setRobotPose(getPose());
        backLeft.putDashboard();
        backRight.putDashboard();
        frontLeft.putDashboard();
        frontRight.putDashboard();
    }
}
