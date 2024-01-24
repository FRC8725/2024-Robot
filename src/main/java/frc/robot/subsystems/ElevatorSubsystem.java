package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ModuleTalonFX;
import frc.robot.RobotMap.ElevatorPort;

public class ElevatorSubsystem extends SubsystemBase {
    private final ModuleTalonFX rightMotor = new ModuleTalonFX(ElevatorPort.kRightMotor);
    private final ModuleTalonFX leftMotor = new ModuleTalonFX(ElevatorPort.kLeftMotor);
    private final DigitalInput limitSwitch = new DigitalInput(0);
    private final PIDController elevatorPidController = new PIDController(0., 0., 0.);

    private final double LIMIT = 999;
    private final double SPEED = 0.2;

    public ElevatorSubsystem() {
        rightMotor.setInverted(false);
        leftMotor.setInverted(false);

        this.resetPosition();
    }

    public void move(boolean direction) {
        if (direction && rightMotor.getRadPosition() < LIMIT && leftMotor.getRadPosition() < LIMIT) {
            rightMotor.set(SPEED);
            leftMotor.set(SPEED);
        } else if (!direction && !limitSwitch.get()) {
            rightMotor.set(-SPEED);
            leftMotor.set(-SPEED);
        }
    }

    public void move(double position) {
        rightMotor.set(elevatorPidController.calculate(rightMotor.getRadPosition(), position));
        leftMotor.set(elevatorPidController.calculate(leftMotor.getRadPosition(), position));
    }

    public void resetPosition() {
        rightMotor.setRadPosition(0);
        leftMotor.setRadPosition(0);
    }

    public void stop() {
        rightMotor.set(0);
        leftMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position right", rightMotor.getRadPosition());
        SmartDashboard.putNumber("Elevator Position left", leftMotor.getRadPosition());
        // This method will be called once per scheduler run
    }
}
