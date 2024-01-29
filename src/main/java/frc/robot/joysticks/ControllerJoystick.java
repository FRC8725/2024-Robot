package frc.robot.joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.UnitTypes;

// TODO complete this Class
public class ControllerJoystick extends XboxController {
    private static final int PORT = 0;
    private static final double MUSHROOM_HEAD_DEADBAND = 0.1;

    public ControllerJoystick() {
        super(PORT);
    }

    public boolean isShootButtonDown() {
        return this.getLeftBumper();
    }

    public boolean isLoadButtonDown() {
        return this.getRightBumper();
    }

    public double getAngleTogglerDirection() {
        return MathUtil.applyDeadband(this.getRightY(), MUSHROOM_HEAD_DEADBAND); 
    }

    public boolean isIntakeButtonDown() {
        return this.getBButton();
    }

    public boolean isElevating() {
        return this.getElevatorDirection() != 0;
    }

    public int getElevatorDirection() {
        if (this.getYButton() && this.getAButton()) {
            return 0;
        } else if (this.getYButton()) {
            return 1;
        } else if (this.getAButton()) {
            return -1;
        } else {
            return 0;
        }
    }
}
