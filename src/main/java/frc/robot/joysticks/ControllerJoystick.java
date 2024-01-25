package frc.robot.joysticks;

import edu.wpi.first.wpilibj.XboxController;

public class ControllerJoystick extends XboxController {
    private static final int PORT = 0;

    public ControllerJoystick() {
        super(PORT);
    }

    public boolean getIntakeButton() {
        return this.getBButton();
    }
}
