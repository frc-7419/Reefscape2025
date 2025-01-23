// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.util.CombinedAlert;

public class WristSubsystem extends SubsystemBase {
  private final TalonFX wristMotor;
  private final CANcoder encoder;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final MotionMagicExpoVoltage motionMagicRequest =
      new MotionMagicExpoVoltage(0).withSlot(0);

  private enum ControlMode {
    MANUAL,
    MOTIONMAGIC
  }

  private ControlMode controlMode = ControlMode.MANUAL;

  private final CombinedAlert positionAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Wrist Out of Range",
          "The encoder value is outside the safe range. Subsystem disabled.");

  private final CombinedAlert velocityAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Wrist Velocity Error",
          "The elevator velocity is outside the safe range. Subsystem disabled.");

  private final CombinedAlert accelerationAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Wrist Acceleration Error",
          "The elevator acceleration is outside the safe range. Subsystem disabled.");

  private final CombinedAlert overheatingAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Wrist Overheating",
          "The elevator motors are overheating. Subsystem disabled.");

  public WristSubsystem() {
    this.wristMotor = new TalonFX(WristConstants.kWristMotorID);
    encoder = new CANcoder(5);
    wristMotor.setPosition(0);
    wristMotor.getConfigurator().apply(WristConstants.kWristTalonFXConfiguration);
    brake();
  }

  public double getVelocity() {
    return wristMotor.getVelocity().getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  public void setPower(double power) {
    wristMotor.set(power);
  }

  public AngularVelocity getVelocityInRadians() {
    return RadiansPerSecond.of(getVelocity());
  }

  public void brake() {
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void coast() {
    wristMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public double getPosition() {
    return encoder.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Rotations", encoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Wrist Velocity", getVelocity());
    SmartDashboard.putNumber("Wrist Encoder Position", getPosition());
    SmartDashboard.putNumber(
        "Wrist Velocity (RotationsPerSecond)",
        wristMotor.getVelocity().getValue().in(RotationsPerSecond));
    SmartDashboard.putNumber(
        "Wrist Temperature (Celsius)", wristMotor.getDeviceTemp().getValue().in(Celsius));
    SmartDashboard.putNumber(
        "Wrist Acceleration (RotationsPerSecondPerSecond)",
        wristMotor.getAcceleration().getValue().in(RotationsPerSecondPerSecond));
  }
}
