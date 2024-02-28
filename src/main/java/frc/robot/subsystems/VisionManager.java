package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.helpers.*;
import frc.lib.helpers.CoordinateSystem;
import frc.lib.math.NotePositionEstimator;
import org.apache.commons.math3.util.FastMath;

@TidiedUp
public class VisionManager extends SubsystemBase implements IDashboardProvider {
    @OutputUnit(UnitTypes.CENTIMETERS)
    private static final double LIMELIGHT_HEIGHT = 60.0;
    @OutputUnit(UnitTypes.DEGREES)
    private static final double LIMELIGHT_MOUNT = -25.0;

    DoubleSubscriber primaryAprilTagId = NetworkTableInstance.getDefault().getTable("limelight")
            .getDoubleTopic("tid").subscribe(-1);
    DoubleSubscriber noteValid = NetworkTableInstance.getDefault().getTable("limelight-note")
            .getDoubleTopic("tv").subscribe(-1);
    DoubleSubscriber noteVerticalAngle = NetworkTableInstance.getDefault().getTable("limelight-note")
            .getDoubleTopic("ty").subscribe(-1);
    DoubleSubscriber noteHorizontalAngle = NetworkTableInstance.getDefault().getTable("limelight-note")
            .getDoubleTopic("tx").subscribe(-1);
    DoubleArraySubscriber blueBotPose = NetworkTableInstance.getDefault().getTable("limelight")
            .getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[7]);
    DoubleArraySubscriber redBotPose = NetworkTableInstance.getDefault().getTable("limelight")
            .getDoubleArrayTopic("botpose_wpired").subscribe(new double[7]);
            

    public VisionManager() {
        this.registerDashboard();
    }

    public Pose2d getVisionRobotPose() {
        final boolean isBlue = DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

        double[] dArr = isBlue ? this.blueBotPose.get() : this.redBotPose.get();
        return new Pose2d(dArr[0], dArr[1], new Rotation2d(Units.degreesToRadians(dArr[5])));
    }

    @CoordinateSystem(CoordinationPolicy.ROBOT_COORDINATION)
    public Translation2d getNotePositionVector() {
        if (this.noNoteTarget()) return null;
        double tx = Units.degreesToRadians(this.noteHorizontalAngle.get());
        double ty = Units.degreesToRadians(this.noteVerticalAngle.get());
        return NotePositionEstimator.getPositionVector(tx, ty);
    }

    public double getNoteHorizontalDistance() {
        if (this.noNoteTarget()) {
            return 0.0;
        }

        double noteOffsetAngle = this.noteVerticalAngle.get();
        double angleToGoal = Units.degreesToRadians(LIMELIGHT_MOUNT + noteOffsetAngle);
        return FastMath.abs(LIMELIGHT_HEIGHT / FastMath.tan(angleToGoal));
    }

    public double getNoteHorizontalAngle() {
        return this.noteHorizontalAngle.get();
    }

    public boolean hasTagTarget() {
        return this.primaryAprilTagId.get() != -1.0;
    }

    public boolean noNoteTarget() {
        return this.noteValid.get() == 0.0;
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("NoteDistance", getNoteHorizontalDistance());
    }

    @Override
    public void putDashboardOnce() {
    }
}
