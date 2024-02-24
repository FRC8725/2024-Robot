package frc.lib;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;

// TODO combine this with ModuleTalonFX
public class ChassisTalonFX extends TalonFX {
    private final double gearRatio;

    public ChassisTalonFX(int motorId, double gearRatio) {
        super(motorId);
        this.setCurrentLimit(false);
        this.gearRatio = gearRatio;
    }

    public void setCurrentLimit(boolean isLimitHigh) {
        double currentLimit = isLimitHigh ? 35.0 : 25.0;
        double thresholdCurrent = isLimitHigh ? 40.0 : 30.0;
        double thresholdTime = 0.2;

        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(currentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(currentLimit)
                .withSupplyCurrentThreshold(thresholdCurrent)
                .withSupplyTimeThreshold(thresholdTime);

        TalonFXConfiguration FXConfig = new TalonFXConfiguration().withCurrentLimits(currentConfig);
        this.getConfigurator().refresh(FXConfig);
    }

    public void setRadPosition(double radian) {
        double rotation = Units.radiansToRotations(radian);
        this.getConfigurator().setPosition(rotation / this.gearRatio);
    }
}
