// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ClawConstants;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.util.CombinedAlert;

public class ClawSubsystem extends SubsystemBase {
  private TalonFX clawMotor;
  private CANcoder absEncoder;
  private DigitalInput beamBreak;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  public ClawSubsystem() {
    this.clawMotor = new TalonFX(Constants.ClawConstants.kClawMotorId);
    this.absEncoder =
        new CANcoder(
            Constants.ClawConstants.kAbsoluteEncoderChannel); // Unkown Error, check internal issue
    this.beamBreak = new DigitalInput(Constants.ClawConstants.kBeambreakid);
    clawMotor.getConfigurator().apply(Constants.ClawConstants.kMotionMagicConfig);
    ;
  }

  private final CombinedAlert positionAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Wrist Out of Range",
          "The claw angle is outside the safe range. Subsystem disabled.");

  private final CombinedAlert velocityAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Wrist Velocity Error",
          "The claw velocity is outside the safe range. Subsystem disabled.");

  private final CombinedAlert overheatingAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Wrist Overheating",
          "The claw motor is overheating. Subsystem disabled.");

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
    clawMotor.setVoltage(power * 12);
  }

  public Boolean getBeamBreak() {
    return beamBreak.get();
  }

  private void toPosition(Angle angle) {
    controlMode = ControlMode.MOTIONMAGIC;

    if (!safetyCheck()) return;

    clawMotor.setControl(
        positionRequest
            .withPosition(angle)
            .withLimitForwardMotion(getPosition().lte(ClawConstants.kMaxPosition))
            .withLimitForwardMotion(getPosition().gte(ClawConstants.kMinPosition)));
  }

  private void switchControlMode(ControlMode control) {
    controlMode = control;
  }

  public Command setPosition(Angle angle) {
    return this.runEnd(() -> toPosition(angle), () -> switchControlMode(ControlMode.MANUAL));
  }

  public void setSpeed(AngularVelocity speed) {
    clawMotor.setControl(
        velocityRequest
            .withVelocity(speed)
            .withLimitForwardMotion(getVelocity().lte(ClawConstants.kMaxSpeed))
            .withLimitForwardMotion(getVelocity().gte(ClawConstants.kMinSpeed)));
  }

  public void coast() {
    clawMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setClosingVoltage(double voltage) {
    clawMotor.setVoltage(voltage);
  }

  public void setOpeningVoltage(double voltage) {
    clawMotor.setVoltage(-voltage);
  }

  public void brake() {
    clawMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public Angle getPosition() {
    return absEncoder.getPosition().getValue();
  }

  public double getPositionDouble() {
    return clawMotor.getPosition().getValueAsDouble();
  }

  public AngularVelocity getVelocity() {
    return clawMotor.getVelocity().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Claw Velocity: ", absEncoder.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Passing Safety Checks: ", safetyCheck());
  }

  private boolean safetyCheck() {
    if (!RobotConstants.runSafetyCheck) return false;

    if (getVelocity().abs(RotationsPerSecond)
        >= ClawConstants.UNSAFE_SPEED.in(RotationsPerSecond)) {
      brake();
      velocityAlert.set(true);
      return false;
    } else velocityAlert.set(false);

    if (clawMotor.getDeviceTemp().getValue().gte(ClawConstants.MAX_TEMPERATURE)) {
      brake();
      overheatingAlert.set(true);
      return false;
    } else {
      overheatingAlert.set(false);
    }

    Angle currentAngle = getPosition();
    if (currentAngle.gt(ClawConstants.kMaxAngle.plus(ClawConstants.kAngleTolerance))
        || currentAngle.lt(ClawConstants.kMinAngle.minus(ClawConstants.kAngleTolerance))) {
      brake();
      positionAlert.set(true);
      return false;
    } else {
      positionAlert.set(false);
      coast();
    }
    return true;
  }
}
