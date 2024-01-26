package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ModuleTalonFX;
import frc.robot.constants.RobotCANPorts;

public class ElevatorSubsystem extends SubsystemBase {
    private static final double LIMIT = 1200;
    private static final double SPEED = 0.2;
    private final ModuleTalonFX rightMotor = new ModuleTalonFX(RobotCANPorts.RIGHT_ELEVATOR.get());
    private final ModuleTalonFX leftMotor = new ModuleTalonFX(RobotCANPorts.LEFT_ELEVATOR.get());
    private final DigitalInput limitSwitch = new DigitalInput(0);
    private final PIDController elevatorPidController = new PIDController(0., 0., 0.); // TODO fill in values

    public ElevatorSubsystem() {
        this.rightMotor.setInverted(true);
        this.leftMotor.setInverted(false);
        this.rightMotor.setNeutralMode(NeutralModeValue.Brake);
        this.leftMotor.setNeutralMode(NeutralModeValue.Brake);
        this.resetPosition();
    }

    public void move(boolean direction) {
        if (direction && rightMotor.getRadPosition() < LIMIT && leftMotor.getRadPosition() < LIMIT) {
            rightMotor.set(SPEED);
            leftMotor.set(SPEED);
        } else if (!direction) {
            rightMotor.set(-SPEED);
            leftMotor.set(-SPEED);
        } else {
            this.stop();
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
        SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
        // This method will be called once per scheduler run
    }
}
