// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.constants.Constants.GroundIntakeWristConstants;
import frc.robot.util.CombinedAlert;

/**
 * The {@code GroundIntakeWristSubsystem} class controls the ground intake wrist of the robot.
 * It manages the motion and angle of the ground intake using a TalonFX motor paired with a fused CANcoder.
 */
public class GroundIntakeWristSubsystem extends SubsystemBase {
  // Hardware devices for the ground intake wrist.
  private final TalonFX intakeWristMotor = new TalonFX(GroundIntakeWristConstants.kGroundIntakeWristMotorID);
  private final CANcoder intakeWristEncoder = new CANcoder(GroundIntakeWristConstants.kGroundIntakeWristEncoderID);

  // Control requests using CTRE's motion magic and velocity control.
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0).withSlot(0);

  /** Enum to keep track of which control mode is active. */
  private enum ControlMode {
    MANUAL,
    MOTIONMAGIC
  }
  private ControlMode controlMode = ControlMode.MANUAL;

  // Alerts for safety monitoring.
  private final CombinedAlert positionAlert = new CombinedAlert(
      CombinedAlert.Severity.ERROR,
      "Ground Intake Wrist Out of Range",
      "The ground intake wrist angle is outside the safe range. Subsystem disabled.");

  private final CombinedAlert velocityAlert = new CombinedAlert(
      CombinedAlert.Severity.ERROR,
      "Ground Intake Wrist Velocity Error",
      "The ground intake wrist velocity is outside the safe range. Subsystem disabled.");

  private final CombinedAlert overheatingAlert = new CombinedAlert(
      CombinedAlert.Severity.ERROR,
      "Ground Intake Wrist Overheating",
      "The ground intake wrist motor is overheating. Subsystem disabled.");

  /** 
   * Creates a new {@code GroundIntakeWristSubsystem} and applies the configuration to both the TalonFX and CANcoder.
   */
  public GroundIntakeWristSubsystem() {
    intakeWristMotor.getConfigurator().apply(GroundIntakeWristConstants.kGroundIntakeWristTalonFXConfiguration);
    intakeWristEncoder.getConfigurator().apply(GroundIntakeWristConstants.kGroundIntakeWristCANCoderConfig);
  }

  /**
   * Sets the ground intake wrist power in manual mode.
   *
   * @param power The power to set (from -1 to 1). Positive values move the wrist in one direction,
   *              negative values move it in the opposite direction.
   */
  public void setPower(double power) {
    // Do not override motion magic commands or if the safety check fails.
    if (controlMode == ControlMode.MOTIONMAGIC || !safetyCheck()) {
      return;
    }

    // Clamp power between -1 and 1.
    power = Math.max(-1, Math.min(1, power));

    intakeWristMotor.setControl(
        velocityRequest
            .withVelocity(power * GroundIntakeWristConstants.kMaxSpeed.in(RotationsPerSecond))
            .withLimitForwardMotion(getPosition().gte(GroundIntakeWristConstants.kMaxAngle))
            .withLimitReverseMotion(getPosition().lte(GroundIntakeWristConstants.kMinAngle)));
  }

  /**
   * Moves the ground intake wrist to a specified angle using Motion Magic.
   *
   * @param angle The desired target angle (e.g., 0° is horizontal, 90° is vertical).
   */
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

  /**
   * Returns a command that drives the ground intake wrist to a specific angle.
   * Once the command ends, the control mode is switched back to MANUAL.
   *
   * @param angle The target angle.
   * @return A command for scheduling.
   */
  public Command setAngle(Angle angle) {
    return this.runEnd(() -> toAngle(angle), () -> switchControlMode(ControlMode.MANUAL));
  }

  /**
   * Returns a command that applies manual ground intake wrist control (e.g., from a joystick).
   *
   * @param power The power from -1 to 1.
   * @return A command for scheduling.
   */
  public Command joystickControl(double power) {
    return this.run(() -> setPower(power));
  }

  /** 
   * Switches the current control mode.
   *
   * @param mode The new control mode.
   */
  private void switchControlMode(ControlMode mode) {
    this.controlMode = mode;
  }

  /** Sets the TalonFX to brake mode (resists motion when no power is applied). */
  public void brake() {
    intakeWristMotor.set(0);
    intakeWristMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /** Sets the TalonFX to coast mode (spins freely when no power is applied). */
  public void coast() {
    intakeWristMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /**
   * Gets the current position of the ground intake wrist.
   *
   * @return The current angle.
   */
  public Angle getPosition() {
    return intakeWristEncoder.getAbsolutePosition().getValue();
  }

  /**
   * Gets the current angular velocity of the ground intake wrist.
   *
   * @return The current angular velocity.
   */
  public AngularVelocity getVelocity() {
    return intakeWristEncoder.getVelocity().getValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ground Intake Wrist Angle (Rotations)", getPosition().in(Rotations));
    SmartDashboard.putNumber("Ground Intake Wrist Angle (Degrees)", getPosition().in(Degrees));
    SmartDashboard.putNumber("Ground Intake Wrist Velocity (RotationsPerSecond)", getVelocity().in(RotationsPerSecond));
    SmartDashboard.putNumber("Ground Intake Wrist Temperature (Celsius)",
        intakeWristMotor.getDeviceTemp().getValue().in(Celsius));
  }

  /**
   * Performs a safety check to ensure the ground intake wrist operates within safe parameters.
   * This includes verifying velocity, temperature, and angle limits. If any of these conditions
   * are violated, the subsystem is disabled (motors are set to brake mode) and an alert is raised.
   *
   * @return {@code true} if operating safely, {@code false} otherwise.
   */
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

    if (intakeWristMotor.getDeviceTemp().getValue().gte(GroundIntakeWristConstants.MAX_TEMPERATURE)) {
      brake();
      overheatingAlert.set(true);
      return false;
    } else {
      overheatingAlert.set(false);
    }

    Angle currentAngle = getPosition();
    if (currentAngle.gt(GroundIntakeWristConstants.kMaxAngle.plus(GroundIntakeWristConstants.kAngleTolerance))
        || currentAngle.lt(GroundIntakeWristConstants.kMinAngle.minus(GroundIntakeWristConstants.kAngleTolerance))) {
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
