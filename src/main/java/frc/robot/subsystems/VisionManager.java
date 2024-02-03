package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.UnitTypes;

// TODO tidy this Class
public class VisionManager extends SubsystemBase {
    private static final VisionManager instance = new VisionManager();
    @OutputUnit(UnitTypes.CENTIMETERS)
    private static final double LIMELIGHT_HEIGHT = 45.0;
    @OutputUnit(UnitTypes.DEGREES)
    private static final double LIMELIGHT_MOUNT = 0;

    DoubleSubscriber tag_tidSub = NetworkTableInstance.getDefault().getTable("limelight").getDoubleTopic("tid").subscribe(-1);
    DoubleArraySubscriber tag_targetpose_robotspaceSub = NetworkTableInstance.getDefault().getTable("limelight").getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);
    DoubleSubscriber note_tvSub = NetworkTableInstance.getDefault().getTable("limelight-note").getDoubleTopic("tv").subscribe(-1);
    DoubleSubscriber note_tySub = NetworkTableInstance.getDefault().getTable("limelight-note").getDoubleTopic("ty").subscribe(-1);
    DoubleSubscriber note_txSub = NetworkTableInstance.getDefault().getTable("limelight-note").getDoubleTopic("tx").subscribe(-1);

    public VisionManager() {
    }

    public static VisionManager getInstance() {
        return instance;
    }

    public Transform3d getAprilTagRelative() {
        var bestCameraToTargetArray = tag_targetpose_robotspaceSub.get();
        Transform3d bestCameraToTarget = new Transform3d();
        if (hasTagTarget()) {
            bestCameraToTarget = new Transform3d(
                    new Translation3d(bestCameraToTargetArray[0], -bestCameraToTargetArray[1], bestCameraToTargetArray[2]),
                    new Rotation3d(bestCameraToTargetArray[3], bestCameraToTargetArray[4], bestCameraToTargetArray[5])
            );
        }
        return bestCameraToTarget;
    }

    public double getNoteDistance() {
        if (!hasNoteTarget()) {
            return 0.0;
        }

        double noteOffsetAngle = note_tySub.get();
        double noteHeight = 0;
        double angleToGoalRadians = Units.degreesToRadians(LIMELIGHT_MOUNT + noteOffsetAngle);
        double distanceToTarget = (noteHeight - LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);

        return distanceToTarget;
    }

    public double getNoteHorizontal() {
        return note_txSub.get();
    }

    public boolean hasTagTarget() {
        return tag_tidSub.get() != -1.;
    }

    public boolean hasSpecTagTarget(int tag) {
        return tag_tidSub.get() == tag;
    }

    public boolean hasNoteTarget() {
        return note_tvSub.get() != 0.0;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("distance", getNoteDistance());
    }
}
