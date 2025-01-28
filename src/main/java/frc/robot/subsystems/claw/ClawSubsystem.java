// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new clawsubsystem. */
  private TalonFX clawMotor;

  private CANcoder absEncoder;
  private DigitalInput beamBreak;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public ClawSubsystem() {
    this.clawMotor = new TalonFX(Constants.ClawConstants.kClawMotorId);
    this.absEncoder =
        new CANcoder(
            Constants.ClawConstants.kAbsoluteEncoderChannel); // Unkown Error, check internal issue
    this.beamBreak = new DigitalInput(Constants.ClawConstants.kBeambreakid);
    clawMotor.getConfigurator().apply(Constants.ClawConstants.kMotionMagicConfig);
    // limitSwitch = new DigitalInput(4);

  }

  // Code stolen from Wrist subsystem
  private enum ControlMode {
    MANUAL,
    MOTIONMAGIC
  }

  private ControlMode controlMode = ControlMode.MANUAL;

  public void setPower(double power) {
    if (controlMode == ControlMode.MOTIONMAGIC || !safetyCheck()) {
      return;
    }

    power = Math.max(-1, Math.min(1, power));

    clawMotor.setControl(
        velocityRequest
            .withVelocity(power * ClawConstants.kMaxSpeed.in(RotationsPerSecond))
            .withLimitForwardMotion(getPosition().lte(ClawConstants.kMaxPosition))
            .withLimitForwardMotion(getPosition().gte(ClawConstants.kMinPosition)));
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

  public Angle getPosition() {
    return absEncoder.getPosition().getValue();
  }

  private boolean safetyCheck() {
    return true; // Not implemented yet
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("AMRIT Claw Velocity is", absEncoder.getVelocity().getValueAsDouble());
  }
}
