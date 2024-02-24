package frc.lib;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.UnitTypes;

// TODO combine this with ChassisTalonFX
public class ModuleTalonFX extends TalonFX {
    public ModuleTalonFX(int motorId) {
        super(motorId);

        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentThreshold(10)
                .withSupplyTimeThreshold(0.5);

        TalonFXConfiguration talonConfig = new TalonFXConfiguration().withCurrentLimits(currentConfig);

        this.getConfigurator().apply(talonConfig);
    }

    @OutputUnit(UnitTypes.RADIANS)
    public double getRadPosition() {
        return Units.rotationsToRadians(this.getPosition().getValue());
    }

    public void setRadPosition(double radian) {
        double rotation = Units.radiansToRotations(radian);
        this.getConfigurator().setPosition(rotation);
    }
}
