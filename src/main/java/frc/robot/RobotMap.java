package frc.robot;

public final class RobotMap {
    public static class SwervePort{
        public static final int kFrontLeftDriveMotor = 2;
        public static final int kBackLeftDriveMotor = 5;
        public static final int kFrontRightDriveMotor = 4;
        public static final int kBackRightDriveMotor = 7;

        public static final int kFrontLeftTurningMotor = 1;
        public static final int kBackLeftTurningMotor = 6;
        public static final int kFrontRightTurningMotor = 3;
        public static final int kBackRightTurningMotor = 8;

        public static final int kFrontLeftDriveAbsEncoder = 9;
        public static final int kBackLeftDriveAbsEncoder = 11;
        public static final int kFrontRightDriveAbsEncoder = 10;
        public static final int kBackRightDriveAbsEncoder = 12;
    }

    public static class ShooterPort{
        public static final int kLeftMotor = 15;
        public static final int kRightMotor = 14;
        public static final int kloadMotor = 13;
        public static final int kLiftMotor = 16;
    }

    public static class  ElevatorPort {
        public static final int kLeftMotor = 17;
        public static final int kRightMotor = 18;
    }
}
