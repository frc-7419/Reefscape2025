// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new clawsubsystem. */
  private TalonFX clawMotor;

  private CANcoder absEncoder;
  // private DigitalInput limitSwitch;

  private DigitalInput beamBreak;

  public ClawSubsystem() {
    this.clawMotor = new TalonFX(Constants.ClawConstants.kClawMotorId);
    this.absEncoder =
        new CANcoder(Constants.ClawConstants.kAbsoluteEncoderChannel); // Unkown Error, check internal issue
    this.beamBreak = new DigitalInput(Constants.ClawConstants.kBeambreakid);
    clawMotor.getConfigurator().apply(Constants.ClawConstants.kMotionMagicConfig);
    // limitSwitch = new DigitalInput(4);

  }

  public void coast() {
    clawMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setClosingVoltage(double speed) {
    clawMotor.set(speed);
  }

  public void setOpeningVoltage(double speed) {
    clawMotor.set(-speed);
  }

  public void brake() {
    clawMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getPose() {
    return absEncoder.getPosition().getValueAsDouble();
  }

  public void setControl(ControlRequest request) {
    absEncoder.setControl(request);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("AMRIT Claw Velocity is", absEncoder.getVelocity().getValueAsDouble());
  }
}
