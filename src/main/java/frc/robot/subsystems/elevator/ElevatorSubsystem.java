// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ElevatorConstants;

// Assuming REV Through Bore Encoder is used for elevator position feedback
// No motion magic unless cancoder :(
public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX leftElevatorMotor = new TalonFX(ElevatorConstants.kLeftElevatorMotorId);
  private TalonFX rightElevatorMotor = new TalonFX(ElevatorConstants.kRightElevatorMotorId);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public TalonFX getLeftElevator() {
    return leftElevatorMotor;
  }

  public TalonFX getRightElevator() {
    return rightElevatorMotor;
  }

}
