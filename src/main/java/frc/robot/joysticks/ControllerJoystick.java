package frc.robot.joysticks;

import edu.wpi.first.wpilibj.XboxController;

public class ControllerJoystick extends XboxController {
    public ControllerJoystick(int port) {
        super(port);
    }

    public boolean getIntakeButton() {
        return this.getBButton();
    }
}
