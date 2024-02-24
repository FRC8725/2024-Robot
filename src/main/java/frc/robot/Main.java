package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.helpers.TidiedUp;

@TidiedUp
public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
