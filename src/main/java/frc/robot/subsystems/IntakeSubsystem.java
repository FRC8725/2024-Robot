// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ModuleTalonFX;
import frc.robot.RobotMap.IntakePort;

public class IntakeSubsystem extends SubsystemBase {
  private final ModuleTalonFX intakeMotor = new ModuleTalonFX(IntakePort.kIntakeMotor);
  private final CANSparkMax liftMotor = new CANSparkMax(IntakePort.kLiftMotor, MotorType.kBrushless);

  private final double IntakeSpeed = 0.07;
  private final double LiftSpeedRate = 0.07;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

  }

  public void intake() {
    intakeMotor.set(IntakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void lift(double LiftSpeed) {
    liftMotor.set(LiftSpeed * LiftSpeedRate);
  }

  public void stopLift() {
    liftMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
