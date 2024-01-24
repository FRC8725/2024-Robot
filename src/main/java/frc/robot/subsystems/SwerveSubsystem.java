package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.SwerveHelper;
import frc.lib.helpers.UnitTypes;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.constants.RobotCANPorts;

public class SwerveSubsystem extends SubsystemBase {
    @OutputUnit(UnitTypes.METERS_PER_SECOND)
    public static final double TELEOP_MAX_ROBOT_SPEED = 2.0;
    @OutputUnit(UnitTypes.RADIANS_PER_SECOND)
    public static final double TELEOP_MAX_ROBOT_ANGULAR_SPEED = Math.PI;
    @OutputUnit(UnitTypes.METERS_PER_SECOND_SQUARED)
    public static final double TELEOP_MAX_ACCELERATION = 6.0;
    @OutputUnit(UnitTypes.RADIANS_PER_SECOND_SQUARED)
    public static final double TELEOP_MAX_ANGULAR_ACCELERATION = 2.0 * Math.PI;
    @OutputUnit(UnitTypes.METERS)
    private static final double TRACK_WIDTH = Units.inchesToMeters(24.0);
    @OutputUnit(UnitTypes.METERS)
    private static final double WHEEL_BASE = Units.inchesToMeters(24.0);

    private final SwerveModule frontLeft = new SwerveModule("FL Module",
            RobotCANPorts.FL_DRIVE.get(),
            RobotCANPorts.FL_STEER.get(),
            true,
            true,
            RobotCANPorts.FL_ABS_ENCODER.get(),
            0.0);
    private final SwerveModule frontRight = new SwerveModule("FR Module",
            RobotCANPorts.FR_DRIVE.get(),
            RobotCANPorts.FR_STEER.get(),
            false,
            true,
            RobotCANPorts.FR_ABS_ENCODER.get(),
            0.0);
    private final SwerveModule backLeft = new SwerveModule("BL Module",
            RobotCANPorts.BL_DRIVE.get(),
            RobotCANPorts.BL_STEER.get(),
            true,
            true,
            RobotCANPorts.BL_ABS_ENCODER.get(),
            0.0);
    private final SwerveModule backRight = new SwerveModule("BR Module",
            RobotCANPorts.BR_DRIVE.get(),
            RobotCANPorts.BR_STEER.get(),
            false,
            true,
            RobotCANPorts.BR_ABS_ENCODER.get(),
            0.0);
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final Field2d field = new Field2d();
    private final SwerveDriveKinematics kinematics = SwerveHelper.constructKinematics(TRACK_WIDTH, WHEEL_BASE);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            this.kinematics, getRotation2d(), this.getModulePositions()
    );
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean hasStarted = false;

    public SwerveSubsystem() {
        this.xLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAX_ACCELERATION);
        this.yLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAX_ACCELERATION);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAX_ANGULAR_ACCELERATION);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                resetEncoders();
            } catch (Exception ignored) {
            }
        }).start();

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getChassisSpeeds,
                this::driveChassis,
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.05, 0.),
                        new PIDConstants(5.0, 0., 0.),
                        SwerveModule.MODULE_MAX_DRIVING_SPEED, // Max module speed, in m/s
                        0.3, // Drive base radius in meters. Distance from robot center to the furthest module.
                        new ReplanningConfig(true, true,
                                5, 5)
                ), () -> DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Red).isPresent()
                , this
        );
    }

    private ChassisSpeeds getChassisSpeeds() {
        return this.kinematics.toChassisSpeeds(this.getModuleStates());
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

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveModule.MODULE_MAX_DRIVING_SPEED);
        this.frontLeft.setDesiredState(desiredStates[0], this.hasStarted);
        this.frontRight.setDesiredState(desiredStates[1], this.hasStarted);
        this.backLeft.setDesiredState(desiredStates[2], this.hasStarted);
        this.backRight.setDesiredState(desiredStates[3], this.hasStarted);
    }

    public void resetStart() {
        this.hasStarted = false;
    }

    public void setStart() {
        this.hasStarted = true;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                this.frontLeft.getPosition(),
                this.frontRight.getPosition(),
                this.backLeft.getPosition(),
                this.backRight.getPosition()
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
        frontLeft.resetRelativeEncoders();
        frontRight.resetRelativeEncoders();
        backLeft.resetRelativeEncoders();
        backRight.resetRelativeEncoders();
    }

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldOriented) {
        constructAndSetModuleStates(xSpeed, ySpeed, rotation, fieldOriented);
    }

    public void move(double xSpeed, double ySpeed, double rotation, boolean fieldOriented) {
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        rotation = turningLimiter.calculate(rotation);
        constructAndSetModuleStates(xSpeed, ySpeed, rotation, fieldOriented);
    }

    private void constructAndSetModuleStates(Double xSpeed, Double ySpeed, Double rotation, boolean fieldOriented) {
        ChassisSpeeds chassisSpeeds = fieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotation);

        this.driveChassis(chassisSpeeds);
    }

    private void driveChassis(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = this.kinematics.toSwerveModuleStates(chassisSpeeds);
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
    }
}
