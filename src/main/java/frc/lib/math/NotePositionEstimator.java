package frc.lib.math;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.helpers.CoordinateSystem;
import frc.lib.helpers.CoordinationPolicy;
import org.apache.commons.math3.geometry.euclidean.threed.*;

public class NotePositionEstimator {
    private static final double TOLERANCE = 0.000329;
    private static final Vector3D CAMERA_POS = new Vector3D(-0.250, -0.250, 0.590);
    private static final Vector3D CENTER_VIEW = new Vector3D(0.203, 0.658, -0.590);
    private static final Vector3D CAMERA_X_AXIS = new Vector3D(0.671, -0.120, 0.097);
    private static final Vector3D CAMERA_Y_AXIS = new Vector3D(-0.007, -0.416, -0.466);
    private static final Plane GROUND = new Plane(new Vector3D(0.0, 0.0, 1.0), TOLERANCE);

    public static Translation2d getPositionVector(double tx, double ty) {
        Rotation xRot = new Rotation(CAMERA_Y_AXIS, tx, RotationConvention.VECTOR_OPERATOR);
        Rotation yRot = new Rotation(CAMERA_X_AXIS, ty, RotationConvention.VECTOR_OPERATOR);
        Vector3D xVector = xRot.applyTo(CENTER_VIEW);
        Vector3D yVector = yRot.applyTo(CENTER_VIEW);
        Plane xPlane = new Plane(CAMERA_POS, CAMERA_POS.add(xVector), CAMERA_POS.add(CAMERA_Y_AXIS), TOLERANCE);
        Plane yPlane = new Plane(CAMERA_POS, CAMERA_POS.add(yVector), CAMERA_POS.add(CAMERA_X_AXIS), TOLERANCE);
        Vector3D intersection = Plane.intersection(xPlane, yPlane, GROUND);
        Translation2d vector = new Translation2d(intersection.getX(), intersection.getY());
        return vector.plus(new Translation2d(0.0, 0.0));
    }
}
