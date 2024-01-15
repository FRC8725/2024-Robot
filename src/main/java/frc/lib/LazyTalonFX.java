package frc.lib;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.util.Units;

public class LazyTalonFX extends TalonFX {
    private final double gearRatio;

    public LazyTalonFX(int deviceNumber, double gearRatio) {
        super(deviceNumber);
        // If the motor is for chassis, we won't set a high current limit.
        this.setCurrentLimit(false);
        this.gearRatio = gearRatio;
    }

    public void setCurrentLimit(boolean isLimitHigh) {
        double currentLimit = isLimitHigh ? 35.0 : 25.0;
        double thresholdCurrent = isLimitHigh ? 40.0 : 30.0;
        double thresholdTime = 0.2;

        var currentConfig = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(currentLimit)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(currentLimit)
            .withSupplyCurrentThreshold(thresholdCurrent)
            .withSupplyTimeThreshold(thresholdTime);

        var FXConfig = new TalonFXConfiguration().withCurrentLimits(currentConfig);
        this.getConfigurator().refresh(FXConfig);

    }

    public void setRadPosition(double radian) {
        double rotation = Units.radiansToRotations(radian);
        this.getConfigurator().setPosition(rotation * (2048.0 / this.gearRatio));
    }

    public double getVelocityAsMPS(double circumference) {
        var rotorVelSingal = this.getRotorVelocity();
        double motorRPM = rotorVelSingal.getValue() * (600.0 / 2048.0);
        double mechRPM = motorRPM * this.gearRatio;
        //rotorVelSingal.waitForUpdate(0.020);
        return mechRPM * circumference / 60.0;
    }

    public double getPositionAsRad() {
        var rotorPosSingal = this.getRotorPosition();
        double radians = Units.rotationsToRadians(rotorPosSingal.getValue());
        // rotorPosSingal.waitForUpdate(0.020);
        return radians / (2048.0 / this.gearRatio);
    }
}
