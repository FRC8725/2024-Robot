package frc.robot.joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.helpers.TidiedUp;

@TidiedUp
public class ControllerJoystick extends XboxController {
    private static final int PORT = 0;
    private static final double MUSHROOM_HEAD_DEADBAND = 0.1;

    public ControllerJoystick() {
        super(PORT);
    }

    public boolean isShootButtonDown() {
        return this.getLeftBumper();
    }

    public boolean isIntakeButtonDown() {
        return this.getBButton();
    }

    public boolean isReleaseButtonDown() {
        return this.getXButton();
    }

    public double getIntakeLiftDirection() {
        return MathUtil.applyDeadband(this.getLeftY(), MUSHROOM_HEAD_DEADBAND);
    }

    public boolean getIntakeToAMP() {
        return this.getRightBumper();
    }

    public boolean isTelescoping() {
        return this.getTelescopeDirection() != 0;
    }

    public int getTelescopeDirection() {
        if (this.getYButton() && this.getAButton()) return 0;
        else if (this.getYButton()) return 1;
        else if (this.getAButton()) return -1;
        return 0;
    }
}
