// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.WristConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeClampSubsystem extends SubsystemBase {
  private TalonFX intakeAlgaeMotor = new TalonFX(intakeId);
  private TalonFX pivotAlgaeMotor = new TalonFX(pivotId);

  public AlgaeClampSubsystem() {
    intakeAlgaeMotor.setPosition(0);
    pivotAlgaeMotor.setPosition(0);
  }

  public void setIntakePower(double power) {
    intakeAlgaeMotor.set(power);
  }

  public void setPivotPower(double power) {
    pivotAlgaeMotor.set(power);
  }

  public void coast() {
    intakeAlgaeMotor.setNeutralMode(NeutralModeValue.Coast);
    pivotAlgaeMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void brake() {
    intakeAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setPivotToAngle(double angle) {
    Angle desiredPosition = Units.Degrees.of(angle);
    pivotAlgaeMotor.setPosition(desiredPosition);
  }

  public void logTelemetry() {
    SmartDashboard.putNumber(
        "Pivot Current Position", pivotAlgaeMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "Intake Current Position", intakeAlgaeMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "Pivot Motor Temperature", pivotAlgaeMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(
        "Intake Motor Temperature", intakeAlgaeMotor.getDeviceTemp().getValueAsDouble());
  }

  @Override
  public void periodic() {
    logTelemetry();
  }
}
