// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSensorSubsystem extends SubsystemBase {
  private final CANrange lightSensor;

  /** Creates a new LightSensorSubsystem. */
  public LightSensorSubsystem(CANrange lightSensor) {
    this.lightSensor = lightSensor;
  }

  public boolean coralThere() {
    return lightSensor.getIsDetected().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
