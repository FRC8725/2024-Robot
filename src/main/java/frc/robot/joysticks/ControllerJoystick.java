package frc.robot.joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.UnitTypes;

// TODO complete this Class
public class ControllerJoystick extends XboxController {
    private static final int PORT = 0;
    private static final double LIFT_DEADBAND = 0.1;

    public ControllerJoystick() {
        super(PORT);
    }

    public boolean isShootButtonDown() {
        return this.getLeftBumper();
    }

    public boolean isLoadButtonDown() {
        return this.getRightBumper();
    }

    @OutputUnit(UnitTypes.PERCENTAGES)
    public double getShooterLiftingSpeed() {
        double speed = this.getRightTriggerAxis() - this.getLeftTriggerAxis();
        return MathUtil.applyDeadband(speed, LIFT_DEADBAND);
    }

    public boolean isShooterLifting() {
        return this.getShooterLiftingSpeed() != 0.0;
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
