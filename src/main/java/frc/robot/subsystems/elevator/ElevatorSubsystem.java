// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.util.CombinedAlert;
import java.util.function.Supplier;

/**
 * The {@code ElevatorSubsystem} class controls the elevator subsystem of the
 * robot. It manages the
 * motion and height of the elevator using TalonFX motors.
 */
public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX leftElevatorMotor = new TalonFX(ElevatorConstants.kLeftElevatorMotorId,
      RobotConstants.kCANivoreBus);
  private final TalonFX rightElevatorMotor = new TalonFX(ElevatorConstants.kRightElevatorMotorId,
      RobotConstants.kCANivoreBus);
  private final TalonFX topElevatorMotor = new TalonFX(ElevatorConstants.kTopElevatorMotorId,
      RobotConstants.kCANivoreBus);

  private final Supplier<Angle> wristAngleSupplier;
  private Angle setpoint;

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
      ElevatorConstants.feedforwardKs,
      ElevatorConstants.feedforwardKg,
      ElevatorConstants.feedforwardKv,
      ElevatorConstants.feedforwardKa);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0).withSlot(0);
  private final SysIdRoutine routine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> leftElevatorMotor.setVoltage(output.in(Volts)), null, this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public enum ControlMode {
    MANUAL,
    MOTIONMAGIC
  }

  private ControlMode controlMode = ControlMode.MANUAL;

  private final CombinedAlert positionAlert = new CombinedAlert(
      CombinedAlert.Severity.ERROR,
      "Elevator Out of Range",
      "The encoder value is outside the safe range. Subsystem disabled.");

  private final CombinedAlert velocityAlert = new CombinedAlert(
      CombinedAlert.Severity.ERROR,
      "Elevator Velocity Error",
      "The elevator velocity is outside the safe range. Subsystem disabled.");

  private final CombinedAlert accelerationAlert = new CombinedAlert(
      CombinedAlert.Severity.ERROR,
      "Elevator Acceleration Error",
      "The elevator acceleration is outside the safe range. Subsystem disabled.");

  private final CombinedAlert overheatingAlert = new CombinedAlert(
      CombinedAlert.Severity.ERROR,
      "Elevator Overheating",
      "The elevator motors are overheating. Subsystem disabled.");

  /** Creates a new {@code ElevatorSubsystem}. */
  public ElevatorSubsystem(Supplier<Angle> wristAngleSupplier) {
    leftElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);
    topElevatorMotor.setPosition(0);

    this.wristAngleSupplier = wristAngleSupplier;

    leftElevatorMotor.getConfigurator().apply(ElevatorConstants.kElevatorTalonFXConfiguration);
    rightElevatorMotor.getConfigurator().apply(ElevatorConstants.kElevatorTalonFXConfiguration);
    topElevatorMotor.getConfigurator().apply(ElevatorConstants.kElevatorTalonFXConfiguration);

    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), false));
    topElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), false));

    brake();
  }

  /**
   * Sets the elevator power in manual mode.
   *
   * @param power The power to set, ranging from -1 to 1. Positive values move the
   *              elevator up, and negative values move it down.
   */
  public void setPower(double power) {
    power = Math.max(-1, Math.min(1, power)) * 8;

    if (getPosition().gt(Rotations.of(1))) {
      power += feedforward.calculate(0);
    }

    if (controlMode == ControlMode.MOTIONMAGIC || !safetyCheck())
      return;
    setVoltage(power);
  }

  public void setVoltage(double voltage) {
    if (!checkMovementSafe(voltage) && voltage != 0) {
      voltage = feedforward.calculate(0);
    }
    leftElevatorMotor.setVoltage(voltage);
    rightElevatorMotor.setVoltage(voltage);
    topElevatorMotor.setVoltage(voltage);
  }

  /**
   * Moves the elevator to the specified position using Motion Magic.
   *
   * @param position The desired position as an Angle.
   */
  public void toPosition(Angle position) {
    controlMode = ControlMode.MOTIONMAGIC;

    if (!safetyCheck())
      return;

    setpoint = position;

    leftElevatorMotor.setControl(
        motionMagicRequest
            .withPosition(position)
            .withLimitForwardMotion(getPosition().gt(ElevatorConstants.kMaxRotations))
            .withLimitReverseMotion(getPosition().lt(ElevatorConstants.kMinRotations)));
  }

  /**
   * Switches the control mode of the elevator.
   *
   * @param controlMode The desired control mode (MANUAL or MOTIONMAGIC).
   */
  public void switchControlMode(ControlMode controlMode) {
    this.controlMode = controlMode;
  }

  /**
   * Returns a command to set the elevator position.
   *
   * @param position The target position as a {@link Distance}.
   * @return The command to execute.
   */
  public Command setPosition(Angle position) {
    return this.runEnd(() -> toPosition(position), () -> switchControlMode(ControlMode.MANUAL));
  }

  public Command joystickControl(CommandXboxController joystick) {
    return this.run(() -> setPower(-joystick.getLeftY()));
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
    topElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /**
   * Sets the motors to brake mode.
   *
   * <p>
   * In brake mode, the motors resist motion when no power is applied.
   */
  public void brake() {
    // Redundancy to ensure all motors are stopped
    leftElevatorMotor.set(0);
    rightElevatorMotor.set(0);
    topElevatorMotor.set(0);

    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    topElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Gets the current elevator encoder position.
   *
   * @return The current encoder position as an {@link Angle}.
   */
  public Angle getPosition() {
    return leftElevatorMotor.getPosition().getValue();
  }

  /** Resets the elevator encoder to zero. */
  public void zeroEncoder() {
    leftElevatorMotor.setPosition(0);
  }

  private boolean checkMovementSafe(double voltage) {
    boolean aboveLimit = getPosition().gt(ElevatorConstants.kElevatorBarUpperLimit);
    boolean belowLimit = getPosition().lt(ElevatorConstants.kElevatorBarLowerLimit);
    boolean wristUnsafe = wristAngleSupplier.get().gt(WristConstants.kElevatorSafeWristAngle);

    if (wristUnsafe && ((belowLimit && voltage > 0) || (aboveLimit && voltage < 0))) {
      return false;
    }

    return true;
  }

  public boolean atPosition() {
    return Math.abs(leftElevatorMotor.getPosition().getValue().minus(setpoint).in(Rotations)) < 0.1;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder Position", getPosition().in(Rotations));
    SmartDashboard.putString("Elevator Control Mode", controlMode.toString());
    SmartDashboard.putBoolean("Disabled", !safetyCheck());
    SmartDashboard.putNumber(
        "Elevator Velocity (RotationsPerSecond)",
        leftElevatorMotor.getVelocity().getValue().in(RotationsPerSecond));
    SmartDashboard.putNumber(
        "Elevator Temperature Left (Celsius)",
        leftElevatorMotor.getDeviceTemp().getValue().in(Celsius));
    SmartDashboard.putNumber(
        "Elevator Temperature Right (Celsius)",
        rightElevatorMotor.getDeviceTemp().getValue().in(Celsius));
    SmartDashboard.putNumber(
        "Elevator Temperature Top (Celsius)",
        topElevatorMotor.getDeviceTemp().getValue().in(Celsius));
    SmartDashboard.putNumber(
        "Elevator Acceleration (RotationsPerSecondPerSecond)",
        leftElevatorMotor.getAcceleration().getValue().in(RotationsPerSecondPerSecond));
  }

  /**
   * Gets the left elevator motor instance.
   *
   * @return The left TalonFX motor instance.
   */
  public TalonFX getLeftMotor() {
    return leftElevatorMotor;
  }

  /**
   * Gets the right elevator motor instance.
   *
   * @return The right TalonFX motor instance.
   */
  public TalonFX getRightMotor() {
    return rightElevatorMotor;
  }

  /**
   * Gets the top elevator motor instance.
   *
   * @return The top TalonFX motor instance.
   */
  public TalonFX getTopMotor() {
    return topElevatorMotor;
  }

  /**
   * Performs a safety check to ensure the elevator operates within safe
   * parameters.
   *
   * <p>
   * This method verifies several safety conditions, including temperature and
   * position
   * limits. If any of these conditions are violated, the subsystem is disabled
   * (motors are set to brake mode),
   * and an appropriate alert is raised. If all conditions are safe, the subsystem
   * remains operational.
   *
   * @return {@code true} if the elevator passes all safety checks and is
   *         operating safely, {@code false} otherwise.
   */
  private boolean safetyCheck() {
    if (!RobotConstants.runSafetyCheck)
      return true;

    if (leftElevatorMotor.getDeviceTemp().getValue().gte(ElevatorConstants.MAX_TEMPERATURE)
        || rightElevatorMotor.getDeviceTemp().getValue().gte(ElevatorConstants.MAX_TEMPERATURE)
        || topElevatorMotor.getDeviceTemp().getValue().gte(ElevatorConstants.MAX_TEMPERATURE)) {
      brake();
      overheatingAlert.set(true);
      return false;
    } else {
      overheatingAlert.set(false);
    }

    if (getPosition().gt(ElevatorConstants.kMaxRotations.plus(ElevatorConstants.OVEREXTENSION_TOLERANCE))
        || getPosition().lt(ElevatorConstants.kMinRotations.minus(ElevatorConstants.OVEREXTENSION_TOLERANCE))) {
      positionAlert.set(true);
      return false;
    } else {
      positionAlert.set(false);
    }
    return true;
  }
}