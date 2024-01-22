package frc.robot.joysticks;

import edu.wpi.first.wpilibj.XboxController;

public class DriverJoystick extends XboxController {
    public static final double DEADBAND = 0.05;

    public DriverJoystick(int port) {
        super(port);
    }
}
