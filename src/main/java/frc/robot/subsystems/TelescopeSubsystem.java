package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ModuleTalonFX;
import frc.lib.helpers.IDashboardProvider;
import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.TidiedUp;
import frc.lib.helpers.UnitTypes;
import frc.robot.constants.RobotPorts;

@TidiedUp
public class TelescopeSubsystem extends SubsystemBase implements IDashboardProvider {
    @OutputUnit(UnitTypes.PERCENTAGES)
    private static final double SPEED = 0.8;

    private final ModuleTalonFX rightMotor = new ModuleTalonFX(RobotPorts.CAN.RIGHT_TELESCOPE.get());
    private final ModuleTalonFX leftMotor = new ModuleTalonFX(RobotPorts.CAN.LEFT_TELESCOPE.get());
    private final Follower follower = new Follower(RobotPorts.CAN.RIGHT_TELESCOPE.get(), true);

    public TelescopeSubsystem() {
        this.rightMotor.setInverted(true);
        this.leftMotor.setInverted(false);
        this.rightMotor.setNeutralMode(NeutralModeValue.Brake);
        this.leftMotor.setNeutralMode(NeutralModeValue.Brake);

        this.resetPosition();

        this.registerDashboard();
    }

    public void move(boolean direction) {
        this.rightMotor.set(SPEED * (direction ? 1 : -1));
        this.leftMotor.setControl(this.follower);
    }

    public void resetPosition() {
        this.rightMotor.setRadPosition(0.0);
        this.leftMotor.setRadPosition(0.0);
    }

    public void stopAll() {
        this.rightMotor.stopMotor();
        this.leftMotor.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Telescope Position right", this.rightMotor.getRadPosition());
        SmartDashboard.putNumber("Telescope Position left", this.leftMotor.getRadPosition());
    }

    @Override
    public void putDashboardOnce() {
    }
}
