package frc.lib.math;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.helpers.CoordinateSystem;
import frc.lib.helpers.CoordinationPolicy;
import org.apache.commons.math3.geometry.euclidean.threed.Plane;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class NotePositionEstimator {
    private static final double TOLERANCE = 0.000329;
    private static final Vector3D CAMERA_POS = new Vector3D(0.17, 0.26, 0.6);
    private static final Vector3D CENTER_VIEW = new Vector3D(0.6993, -0.2179, -0.6);
    private static final Vector3D CAMERA_X_AXIS = new Vector3D(-0.1169, -0.7127, 0.1227);
    private static final Vector3D CAMERA_Y_AXIS = new Vector3D(-0.4543, -0.0157, -0.5238);
    private static final Plane GROUND = new Plane(new Vector3D(0.0, 0.0, 1.0), TOLERANCE);

    @CoordinateSystem(CoordinationPolicy.ROBOT_COORDINATION)
    public static Translation2d getPositionVector(double tx, double ty) {
        Rotation xRot = new Rotation(CAMERA_Y_AXIS, tx, RotationConvention.VECTOR_OPERATOR);
        Rotation yRot = new Rotation(CAMERA_X_AXIS, ty, RotationConvention.VECTOR_OPERATOR);
        Vector3D xVector = xRot.applyTo(CENTER_VIEW);
        Vector3D yVector = yRot.applyTo(CENTER_VIEW);
        Plane xPlane = new Plane(CAMERA_POS, CAMERA_POS.add(xVector), CAMERA_POS.add(CAMERA_Y_AXIS), TOLERANCE);
        Plane yPlane = new Plane(CAMERA_POS, CAMERA_POS.add(yVector), CAMERA_POS.add(CAMERA_X_AXIS), TOLERANCE);
        Vector3D intersection = Plane.intersection(xPlane, yPlane, GROUND);
        Vector2D vector2D = new Vector2D(intersection.getX(), intersection.getY());

        if (vector2D.getNorm() == 0) return new Translation2d(0.0, 0.0);
        vector2D.scalarMultiply((vector2D.getNorm() - 0.415) / vector2D.getNorm());
        return new Translation2d(vector2D.getX(), vector2D.getY());
    }
}
