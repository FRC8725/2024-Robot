package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ModuleTalonFX;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotMap.ShooterPort;

public class ShooterSubsystem extends SubsystemBase {
    private final ModuleTalonFX rightMotor = new ModuleTalonFX(ShooterPort.kRightMotor);
    private final ModuleTalonFX leftMotor = new ModuleTalonFX(ShooterPort.kLeftMotor);
    private final ModuleTalonFX loadMotor = new ModuleTalonFX(ShooterPort.kloadMotor);

    private final ModuleTalonFX liftMotor = new ModuleTalonFX(ShooterPort.kLiftMotor);
    private final Encoder liftEncoder = new Encoder(0, 1);


    public ShooterSubsystem() {
        rightMotor.setInverted(true);
        leftMotor.setInverted(false);
        loadMotor.setInverted(false);
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
