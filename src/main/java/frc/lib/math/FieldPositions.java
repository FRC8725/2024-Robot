package frc.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;


@SuppressWarnings("unused")
public final class FieldPositions {
    public static final Translation2d BLUE_SHOOTER = new Translation2d(0.0, 218.42).times(Units.inchesToMeters(1.0));
    public static final Translation2d RED_SHOOTER = new Translation2d(652.73, 218.42).times(Units.inchesToMeters(1.0));
    public static final Pose2d BLUE_AMP_POSITION = new Pose2d(1.84, 8.07 - 0.10, Rotation2d.fromDegrees(-90.0));
    public static final Pose2d RED_AMP_POSITION = new Pose2d(14.701, 8.07 - 0.10, Rotation2d.fromDegrees(-90.0));
}
