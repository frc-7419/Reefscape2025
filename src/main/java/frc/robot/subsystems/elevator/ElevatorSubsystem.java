// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ElevatorConstants;

// Assuming REV Through Bore Encoder is used for elevator position feedback
// No motion magic unless cancoder :(

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX leftElevatorMotor = new TalonFX(ElevatorConstants.kLeftElevatorMotorId);
  private final TalonFX rightElevatorMotor = new TalonFX(ElevatorConstants.kRightElevatorMotorId);
  private final DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(ElevatorConstants.kElevatorEncoderPort);
  private final ElevatorFeedforward elevatorFeedforward;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));
    elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG,
        ElevatorConstants.kV, ElevatorConstants.kA);
  }

  /**
   * Sets the power for the elevator motors while respecting soft limits.
   *
   * @param power The desired motor power, ranging from -1.0 to 1.0.
   */
  public void setElevatorPower(double power) {
    if (power > 0 && getElevatorHeight().gt(ElevatorConstants.kUpperSoftLimit)) {
      power = 0;
      brake();
    } else if (power < 0 && getElevatorHeight().lt(ElevatorConstants.kLowerSoftLimit)) {
      power = 0;
      brake();
    } else {
      coast();
      leftElevatorMotor.set(power + elevatorFeedforward.calculate(0));
    }
  }

  /**
   * Sets the motors to coast mode.
   *
   * <p>
   * In coast mode, the motors will spin freely when no power is applied.
   */
  public void coast() {
    leftElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /**
   * Sets the motors to brake mode.
   *
   * <p>
   * In brake mode, the motors resist motion when no power is applied.
   */
  public void brake() {
    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Gets the current elevator encoder position.
   *
   * @return The current encoder position as an {@link Angle}.
   */
  public Angle getElevatorEncoder() {
    return Rotations.of(
        elevatorEncoder.get() + ElevatorConstants.kElevatorEncoderOffset.in(Rotations));
  }

  /**
   * Gets the current elevator height based on the encoder value.
   *
   * @return The current height of the elevator as a {@link Distance}.
   */
  public Distance getElevatorHeight() {
    return Meters.of(getElevatorEncoder().in(Rotations) * ElevatorConstants.kRotationToMetersRatio);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Gets the left elevator motor instance.
   *
   * @return The left TalonFX motor instance.
   */
  public TalonFX getLeftElevator() {
    return leftElevatorMotor;
  }

  /**
   * Gets the right elevator motor instance.
   *
   * @return The right TalonFX motor instance.
   */
  public TalonFX getRightElevator() {
    return rightElevatorMotor;
  }
}
