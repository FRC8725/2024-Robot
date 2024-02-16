package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.UnitTypes;
import org.apache.commons.math3.util.FastMath;

// TODO tidy this Class
public class VisionManager extends SubsystemBase {
    @OutputUnit(UnitTypes.CENTIMETERS)
    private static final double LIMELIGHT_HEIGHT = 45.0;
    @OutputUnit(UnitTypes.DEGREES)
    private static final double LIMELIGHT_MOUNT = 0.0;

    DoubleSubscriber primaryAprilTagId = NetworkTableInstance.getDefault().getTable("limelight").getDoubleTopic("tid").subscribe(-1);
    DoubleArraySubscriber primaryAprilTagTransform = NetworkTableInstance.getDefault().getTable("limelight").getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);
    DoubleSubscriber noteValid = NetworkTableInstance.getDefault().getTable("limelight-note").getDoubleTopic("tv").subscribe(-1);
    DoubleSubscriber noteVerticalAngle = NetworkTableInstance.getDefault().getTable("limelight-note").getDoubleTopic("ty").subscribe(-1);
    DoubleSubscriber noteHorizontalAngle = NetworkTableInstance.getDefault().getTable("limelight-note").getDoubleTopic("tx").subscribe(-1);

    public Transform3d getAprilTagRelative() {
        double[] bestCameraToTargetArray = this.primaryAprilTagTransform.get();
        Transform3d bestCameraToTarget = new Transform3d();
        if (this.hasTagTarget()) {
            bestCameraToTarget = new Transform3d(
                    new Translation3d(bestCameraToTargetArray[0], -bestCameraToTargetArray[1], bestCameraToTargetArray[2]),
                    new Rotation3d(bestCameraToTargetArray[3], bestCameraToTargetArray[4], bestCameraToTargetArray[5])
            );
        }

        return bestCameraToTarget;
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

    public boolean hasSpecTagTarget(int tag) {
        return this.primaryAprilTagId.get() == tag;
    }

    public boolean noNoteTarget() {
        return this.noteValid.get() == 0.0;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("distance", getNoteDistance());
    }
}
