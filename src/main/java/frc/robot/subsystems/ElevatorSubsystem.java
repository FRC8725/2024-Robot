// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ModuleTalonFX;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotMap.ElevatorPort;

public class ElevatorSubsystem extends SubsystemBase {
  private final ModuleTalonFX rightMotor = new ModuleTalonFX(ElevatorPort.kRightMotor);
  private final ModuleTalonFX leftMotor = new ModuleTalonFX(ElevatorPort.kLeftMotor);
  private final DigitalInput limitSwitch = new DigitalInput(0);
  private final PIDController elevatorPidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    rightMotor.setInverted(true);
    leftMotor.setInverted(false);

    rightMotor.setRadPosition(0);
    leftMotor.setRadPosition(0);
  }

  public void move(boolean direction) {
    if (direction && rightMotor.getRadPosition() < ElevatorConstants.kHeightLimit && leftMotor.getRadPosition() < ElevatorConstants.kHeightLimit) {
      rightMotor.set(ElevatorConstants.kSpeed);
      leftMotor.set(ElevatorConstants.kSpeed);
    }
    else if (!direction && !limitSwitch.get()) {
      rightMotor.set(-ElevatorConstants.kSpeed);
      leftMotor.set(-ElevatorConstants.kSpeed);
    }
  }
  
  public void move(double position) {
    elevatorPidController.setSetpoint(position);
    rightMotor.set(elevatorPidController.calculate(rightMotor.getRadPosition()));
    leftMotor.set(elevatorPidController.calculate(leftMotor.getRadPosition()));
  }

  public void stop() {
    rightMotor.set(0);
    leftMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
