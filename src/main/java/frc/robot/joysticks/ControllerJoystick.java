package frc.robot.joysticks;

import edu.wpi.first.wpilibj.XboxController;

public class ControllerJoystick extends XboxController {
    public static final double DEADBAND = 0.05;

    public ControllerJoystick(int port) {
        super(port);
    }

    public boolean getIntakeButton() {
        return getBButton();
    }
}
