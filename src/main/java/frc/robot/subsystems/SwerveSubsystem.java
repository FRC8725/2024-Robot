package frc.robot.subsystems;

import org.apache.commons.math3.util.FastMath;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.helpers.*;
import frc.lib.motors.SwerveModuleGroup;
import frc.robot.constants.RobotCANPorts;
import frc.robot.constants.SwerveDriveConstants;

@TidiedUp
public class SwerveSubsystem extends SubsystemBase implements IDashboardProvider {
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
            0.0, new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0));

    private final SwerveModule frontRight = new SwerveModule("FR Module",
            RobotCANPorts.FR_DRIVE.get(),
            RobotCANPorts.FR_STEER.get(),
            false,
            true,
            RobotCANPorts.FR_ABS_ENCODER.get(),
            0.0, new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    private final SwerveModule backLeft = new SwerveModule("BL Module",
            RobotCANPorts.BL_DRIVE.get(),
            RobotCANPorts.BL_STEER.get(),
            true,
            true,
            RobotCANPorts.BL_ABS_ENCODER.get(),
            0.0, new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0));

    private final SwerveModule backRight = new SwerveModule("BR Module",
            RobotCANPorts.BR_DRIVE.get(),
            RobotCANPorts.BR_STEER.get(),
            false,
            true,
            RobotCANPorts.BR_ABS_ENCODER.get(),
            0.0, new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    private final SwerveModuleGroup modules = new SwerveModuleGroup(
            this.frontLeft, this.frontRight, this.backLeft, this.backRight);
    private final SwerveDriveKinematics kinematics = this.modules.constructKinematics();
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            this.kinematics, this.getHeading(), this.getModulePositions(), new Pose2d());
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    // private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);
    private final Field2d gameFieldSim = new Field2d();

    private final ProfiledPIDController drivePIDController =
            new ProfiledPIDController(0.03, 0.0, 0.0,
                    new TrapezoidProfile.Constraints(
                            SwerveDriveConstants.AUTO_MAX_ROBOT_SPEED,
                            SwerveDriveConstants.AUTO_MAX_ACCELERATION
                    ), 0.01
            ); // TODO re-tune PID
    private final ProfiledPIDController steerPIDController =
            new ProfiledPIDController(0.02, 0, 0,
                    new TrapezoidProfile.Constraints(
                            SwerveDriveConstants.AUTO_MAX_ROBOT_ANGULAR_SPEED,
                            SwerveDriveConstants.AUTO_MAX_ANGULAR_ACCELERATION
                    ), 0.01
            ); // TODO re-tune PID

    public SwerveSubsystem() {
        this.registerDashboard();
        this.initialize();
        this.configureAutoBuilder();
    }

    private void initialize() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                this.resetGyro();
                this.resetRelativeEncoders();
            } catch (Exception ignored) {
            }
        }).start();
    }

    private void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
                this::getRobotPosition,
                this::resetOdometry,
                this::getChassisSpeeds,
                this::drive,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0.05, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0),
                        SwerveModule.MODULE_MAX_DRIVING_SPEED,
                        FastMath.sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE),
                        new ReplanningConfig(false, false)
                ), () -> DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Red).isPresent()
                , this
        );
    }

    @CoordinateSystem(CoordinationPolicy.ROBOT_COORDINATION)
    private ChassisSpeeds getChassisSpeeds() {
        return this.kinematics.toChassisSpeeds(this.getModuleStates());
    }

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldOriented) {
        ChassisSpeeds chassisSpeeds = fieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.getHeading())
                : new ChassisSpeeds(xSpeed, ySpeed, rotation);

        this.drive(chassisSpeeds);
    }

    public void drive(SwerveModuleState[] states) {
        this.modules.setDesiredStates(states);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] desiredStates = this.kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveModule.MODULE_MAX_DRIVING_SPEED);
        this.drive(desiredStates);
    }

    @OutputUnit(UnitTypes.DEGREES)
    public double getGyroAngle() {
        return Math.IEEEremainder(-this.gyro.getAngle(), 360);
    }

    public void resetToMiddlePose() {
        this.resetGyro();

        Pose2d pose2d = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
            ? new Pose2d(1.32, 5.55, new Rotation2d())
            : new Pose2d(15.24, 5.55, new Rotation2d(Math.PI));

        this.resetOdometry(pose2d);
    }

    public void resetGyro() {
        this.gyro.reset();
    }

    public void zeroRobotHeading() {
        this.situateRobot(0.0);
    }

    public void situateRobot(double angleSetpoint) {
        this.situateRobot(this.getRobotPosition().rotateBy(Rotation2d.fromRadians(angleSetpoint)));
    }

    public void situateRobot(Pose2d targetPose) {
        double initialRotation = this.getRobotPosition().getRotation().getRadians();
        double targetRotation = targetPose.getRotation().getRadians();

        Translation2d vector = targetPose.getTranslation().minus(this.getRobotPosition().getTranslation());
        double speed = this.drivePIDController.calculate(vector.getNorm(), 0.0);
        double rotation = this.steerPIDController.calculate(initialRotation, targetRotation);

        double xSpeed = speed * vector.getX() / vector.getNorm();
        double ySpeed = speed * vector.getY() / vector.getNorm();
        this.drive(xSpeed, ySpeed, rotation, true);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(this.getGyroAngle());
    }

    @OutputUnit(UnitTypes.METERS)
    public Pose2d getRobotPosition() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public SwerveModuleState[] getModuleStates() {
        return this.modules.getStates();
    }

    public SwerveModulePosition[] getModulePositions() {
        return this.modules.getPositions();
    }

    public void resetOdometry(Pose2d pose) {
        this.poseEstimator.resetPosition(this.getHeading(), this.getModulePositions(), pose);
    }

    public void calibratePose(Pose2d pose2d) {
        this.poseEstimator.addVisionMeasurement(pose2d, Timer.getFPGATimestamp());
    }

    public void resetRelativeEncoders() {
        this.modules.forEach(SwerveModule::resetRelativeEncoders);
    }

    public void stopModules() {
        this.modules.forEach(SwerveModule::stopAll);
    }

    @Override
    public void periodic() {
        this.poseEstimator.update(this.getHeading(), this.getModulePositions());
        this.gameFieldSim.setRobotPose(this.getRobotPosition());
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("RobotHeading", this.getGyroAngle());
        SmartDashboard.putData(this.gameFieldSim);
        SmartDashboard.putString("RobotPose", this.getRobotPosition().toString());
    }

    @Override
    public void putDashboardOnce() {
    }
}
