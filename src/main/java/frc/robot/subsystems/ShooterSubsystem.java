package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotMap.ShooterPort;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX rightMotor = new TalonFX(ShooterPort.kRightMotor);
    private final TalonFX leftMotor = new TalonFX(ShooterPort.kLeftMotor);
    private final CANSparkMax loadMotor = new CANSparkMax(ShooterPort.kloadMotor, MotorType.kBrushless);

    public ShooterSubsystem() {
        var currentConfig = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withStatorCurrentLimit(40).withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(30).withSupplyCurrentThreshold(10).withSupplyTimeThreshold(0.5);

        var talonConfig = new TalonFXConfiguration().withCurrentLimits(currentConfig);

        rightMotor.getConfigurator().apply(talonConfig);
        leftMotor.getConfigurator().apply(talonConfig);

        rightMotor.setInverted(true);
        leftMotor.setInverted(false);

        this.loadMotor.setSmartCurrentLimit(30);
        this.loadMotor.setIdleMode(IdleMode.kCoast);
        this.loadMotor.setInverted(false);
    }

    public void shoot() {
        rightMotor.set(ShooterConstants.kShootSpeed);
        leftMotor.set(ShooterConstants.kShootSpeed);
        loadMotor.set(ShooterConstants.kLoadSpeed);
    }

    public void stop() {
        rightMotor.set(0);
        leftMotor.set(0);
        loadMotor.set(0);
    }

    public void load() {
        loadMotor.set(ShooterConstants.kLoadSpeed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
