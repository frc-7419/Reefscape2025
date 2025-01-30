// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class GroundIntakeSubsystem extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private DigitalInput beamBreak;
  /** Creates a new AlgaeIntakeSubsystem. */
  public GroundIntakeSubsystem() {
    leftMotor = new TalonFX(Constants.GroundIntakeConstants.kLeftIntakeMotorId);
    rightMotor = new TalonFX(Constants.GroundIntakeConstants.kRightIntakeMotorId);
    beamBreak = new DigitalInput(Constants.GroundIntakeConstants.kBeambreakId);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left intake motor current speed:", leftMotor.get());
    SmartDashboard.putNumber("Right intake motor current speed:", rightMotor.get());
    SmartDashboard.putBoolean("Beam Break Triggered:", beamBreakIsTriggered());
  }

  public boolean beamBreakIsTriggered() {
    return !beamBreak.get();
  }

  public void setSpeed(double speed) {
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }

  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  public void brake() {
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void coast() {
    leftMotor.setNeutralMode(NeutralModeValue.Coast);
    rightMotor.setNeutralMode(NeutralModeValue.Coast);
  }
}
