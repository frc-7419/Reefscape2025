// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.groundintake;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.GroundIntakeWristConstants;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.util.CombinedAlert;

public class GroundIntakeWristSubsystem extends SubsystemBase {

  private final TalonFX intakeWristMotor =
      new TalonFX(GroundIntakeWristConstants.kGroundIntakeWristMotorID);
  private final CANcoder intakeWristEncoder =
      new CANcoder(GroundIntakeWristConstants.kGroundIntakeWristEncoderID);

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
          "Ground Intake Wrist Out of Range",
          "The ground intake wrist angle is outside the safe range. Subsystem disabled.");

  private final CombinedAlert velocityAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Ground Intake Wrist Velocity Error",
          "The ground intake wrist velocity is outside the safe range. Subsystem disabled.");

  private final CombinedAlert overheatingAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Ground Intake Wrist Overheating",
          "The ground intake wrist motor is overheating. Subsystem disabled.");

  public GroundIntakeWristSubsystem() {
    intakeWristMotor
        .getConfigurator()
        .apply(
            GroundIntakeWristConstants.kGroundIntakeWristTalonFXConfiguration.withSlot0(
                GroundIntakeWristConstants.kGroundIntakeWristSlot0Configs));
    intakeWristEncoder
        .getConfigurator()
        .apply(GroundIntakeWristConstants.kGroundIntakeWristCANCoderConfig);
  }

  public void setPower(double power) {
    if (controlMode == ControlMode.MOTIONMAGIC || !safetyCheck()) {
      return;
    }

    power = Math.max(-1, Math.min(1, power));

    intakeWristMotor.setControl(
        velocityRequest
            .withVelocity(power * GroundIntakeWristConstants.kMaxSpeed.in(RotationsPerSecond))
            .withLimitForwardMotion(getPosition().gte(GroundIntakeWristConstants.kMaxAngle))
            .withLimitReverseMotion(getPosition().lte(GroundIntakeWristConstants.kMinAngle)));
  }

  private void toAngle(Angle angle) {
    controlMode = ControlMode.MOTIONMAGIC;

    if (!safetyCheck()) {
      return;
    }

    intakeWristMotor.setControl(
        motionMagicRequest
            .withPosition(angle.in(Rotations))
            .withLimitForwardMotion(getPosition().gte(GroundIntakeWristConstants.kMaxAngle))
            .withLimitReverseMotion(getPosition().lte(GroundIntakeWristConstants.kMinAngle)));
  }

  public Command setAngle(Angle angle) {
    return this.runEnd(() -> toAngle(angle), () -> switchControlMode(ControlMode.MANUAL));
  }

  public Command joystickControl(double power) {
    return this.run(() -> setPower(power));
  }

  private void switchControlMode(ControlMode mode) {
    this.controlMode = mode;
  }

  public void brake() {
    intakeWristMotor.setVoltage(0);
    intakeWristMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void coast() {
    intakeWristMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public Angle getPosition() {
    return intakeWristEncoder.getAbsolutePosition().getValue();
  }

  public AngularVelocity getVelocity() {
    return intakeWristEncoder.getVelocity().getValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ground Intake Wrist Angle (Rotations)", getPosition().in(Rotations));
    SmartDashboard.putNumber("Ground Intake Wrist Angle (Degrees)", getPosition().in(Degrees));
    SmartDashboard.putNumber(
        "Ground Intake Wrist Velocity (RotationsPerSecond)", getVelocity().in(RotationsPerSecond));
    SmartDashboard.putNumber(
        "Ground Intake Wrist Temperature (Celsius)",
        intakeWristMotor.getDeviceTemp().getValue().in(Celsius));
  }

  private boolean safetyCheck() {
    if (!RobotConstants.runSafetyCheck) {
      return true;
    }

    if (getVelocity().abs(RotationsPerSecond)
        >= GroundIntakeWristConstants.UNSAFE_SPEED.in(RotationsPerSecond)) {
      brake();
      velocityAlert.set(true);
      return false;
    } else {
      velocityAlert.set(false);
    }

    if (intakeWristMotor
        .getDeviceTemp()
        .getValue()
        .gte(GroundIntakeWristConstants.MAX_TEMPERATURE)) {
      brake();
      overheatingAlert.set(true);
      return false;
    } else {
      overheatingAlert.set(false);
    }

    Angle currentAngle = getPosition();
    if (currentAngle.gt(
            GroundIntakeWristConstants.kMaxAngle.plus(GroundIntakeWristConstants.kAngleTolerance))
        || currentAngle.lt(
            GroundIntakeWristConstants.kMinAngle.minus(
                GroundIntakeWristConstants.kAngleTolerance))) {
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
