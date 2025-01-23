// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristIntakeSubsystem extends SubsystemBase {
  private final TalonFX motorCoral;
  private final TalonFX motorAlgae;

  /** Creates a new WristIntakeSubsystem. */
  public WristIntakeSubsystem(TalonFX motorCoral, TalonFX motorAlgae) {
    this.motorCoral = motorCoral;
    this.motorAlgae = motorAlgae;
  }
  /*
  check if legal to carry both coral & algae together
  if legal, combine coast, brake, etc.
  */ 
  
  public void coastCoral() {
    motorCoral.setNeutralMode(NeutralModeValue.Coast);
  }

  public void coastAlgae() {
    motorAlgae.setNeutralMode(NeutralModeValue.Coast);
  }

  public void brakeCoral() {
    motorCoral.setNeutralMode(NeutralModeValue.Brake);
  }
  
  public void brakeAlgae() {
    motorAlgae.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setCoralPower(double power) {
    motorCoral.set(power);
  }
  
  public void setAlgaePower(double power) {
    motorAlgae.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Motor Algae Speed: ", motorAlgae.get());
    SmartDashboard.putNumber("Motor Coral Speed: ", motorCoral.get());
  }
}
