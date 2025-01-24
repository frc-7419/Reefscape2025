// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new clawsubsystem. */
  private TalonFX clawMotor;

  private DutyCycleEncoder DutyCycleEncoder;

  private DigitalInput beamBreak;

  public ClawSubsystem() {
    this.clawMotor = new TalonFX(Constants.ClawConstants.kClawMotorId);
    this.DutyCycleEncoder = new DutyCycleEncoder(Constants.ClawConstants.kDutyEncoderChannel);
    this.beamBreak = new DigitalInput(Constants.ClawConstants.kBeambreakid);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
