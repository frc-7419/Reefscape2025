// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hangersubsystem extends SubsystemBase {
  /** Creates a new hangersubsystem. */
  private final SparkMax hangerMotor;
  public hangersubsystem() {
    this.hangerMotor = new SparkMax(10, MotorType.kBrushless);

  }

  public void set(double speed){
    hangerMotor.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
