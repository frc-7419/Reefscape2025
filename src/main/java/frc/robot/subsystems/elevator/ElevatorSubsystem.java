// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Explain this code:
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevator/Elevator.java

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.util.CombinedAlert;

/**
 * The {@code ElevatorSubsystem} class controls the elevator subsystem of the
 * robot. It manages the
 * motion and height of the elevator using TalonFX motors.
 */
public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX leadElevatorMotor = new TalonFX(ElevatorConstants.kLeftElevatorMotorId,
      RobotConstants.kCANivoreBus);
  private final TalonFX rightElevatorMotor = new TalonFX(ElevatorConstants.kRightElevatorMotorId,
      RobotConstants.kCANivoreBus);
  private final TalonFX topElevatorMotor = new TalonFX(ElevatorConstants.kTopElevatorMotorId,
      RobotConstants.kCANivoreBus);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0).withSlot(0);
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);
  private final SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> leadElevatorMotor.setVoltage(output.in(Volts)), null, this));
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  private enum ControlMode {
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
  public ElevatorSubsystem() {
    leadElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);
    topElevatorMotor.setPosition(0);

    leadElevatorMotor.getConfigurator().apply(ElevatorConstants.kElevatorTalonFXConfiguration);
    rightElevatorMotor.getConfigurator().apply(ElevatorConstants.kElevatorTalonFXConfiguration);
    topElevatorMotor.getConfigurator().apply(ElevatorConstants.kElevatorTalonFXConfiguration);

    rightElevatorMotor.setControl(new Follower(leadElevatorMotor.getDeviceID(), false));
    topElevatorMotor.setControl(new Follower(leadElevatorMotor.getDeviceID(), false));

    coast();
  }

  /**
   * Sets the elevator power in manual mode.
   *
   * @param power The power to set, ranging from -1 to 1. Positive values move the
   *              elevator up, and
   *              negative values move it down.
   */
  public void setPower(double power) {
    // if (controlMode == ControlMode.MOTIONMAGIC || !safetyCheck()) return;
    power = Math.max(-1, Math.min(1, power));
    leadElevatorMotor.set(power);
    rightElevatorMotor.set(power);
    topElevatorMotor.set(power);
    /*
     * leadElevatorMotor.setControl(
     * velocityRequest
     * .withVelocity(
     * power
     * ElevatorConstants.kMaxSpeed.in(MetersPerSecond)
     * / ElevatorConstants.kMetersPerRotation)
     * .withLimitForwardMotion(getHeight().gt(ElevatorConstants.kMaxHeight))
     * .withLimitReverseMotion(getHeight().lt(ElevatorConstants.kMinHeight)));
     */
  }

  public void setVoltage(double voltage) {
    leadElevatorMotor.setVoltage(voltage);
    rightElevatorMotor.setVoltage(voltage);
    topElevatorMotor.setVoltage(voltage);
  }

  /**
   * Moves the elevator to the specified position using Motion Magic.
   *
   * @param position The desired position in meters.
   */
  private void toPosition(double position) {
    controlMode = ControlMode.MOTIONMAGIC;

    if (!safetyCheck())
      return;

    leadElevatorMotor.setControl(
        motionMagicRequest
            .withPosition(position / ElevatorConstants.kMetersPerRotation)
            .withLimitForwardMotion(getHeight().gt(ElevatorConstants.kMaxHeight))
            .withLimitReverseMotion(getHeight().lt(ElevatorConstants.kMinHeight)));
  }

  /**
   * Switches the control mode of the elevator.
   *
   * @param controlMode The desired control mode (MANUAL or MOTIONMAGIC).
   */
  private void switchControlMode(ControlMode controlMode) {
    this.controlMode = controlMode;
  }

  /**
   * Returns a command to set the elevator position.
   *
   * @param position The target position as a {@link Distance}.
   * @return The command to execute.
   */
  public Command setPosition(Distance position) {
    return this.runEnd(
        () -> toPosition(position.in(Meters) / ElevatorConstants.kMetersPerRotation),
        () -> switchControlMode(ControlMode.MANUAL));
  }

  public Command joystickControl(CommandXboxController joystick) {
    return this.run(() -> setPower(joystick.getLeftY()));
  }

  /**
   * Sets the motors to coast mode.
   *
   * <p>
   * In coast mode, the motors will spin freely when no power is applied.
   */
  public void coast() {
    leadElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
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
    // Reduncancy to ensure both motors are stopped
    leadElevatorMotor.set(0);
    rightElevatorMotor.set(0);
    topElevatorMotor.set(0);

    leadElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    topElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Gets the current elevator encoder position.
   *
   * @return The current encoder position as an {@link Angle}.
   */
  public Angle getPosition() {
    return leadElevatorMotor.getPosition().getValue();
  }

  /** Resets the elevator encoder to zero. */
  public void zeroEncoder() {
    leadElevatorMotor.setPosition(0);
  }

  /**
   * Gets the current elevator height based on the encoder value.
   *
   * @return The current height of the elevator as a {@link Distance}.
   */
  public Distance getHeight() {
    return Meters.of(getPosition().in(Rotations) * ElevatorConstants.kMetersPerRotation)
        .plus(ElevatorConstants.kHeightOffset);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height (Meters)", getHeight().in(Meters));
    SmartDashboard.putNumber("Elevator Encoder Position", getPosition().in(Rotations));
    SmartDashboard.putString("Elevator Control Mode", controlMode.toString());
    SmartDashboard.putBoolean("Disabled", !safetyCheck());
    SmartDashboard.putNumber(
        "Elevator Velocity (RotationsPerSecond)",
        leadElevatorMotor.getVelocity().getValue().in(RotationsPerSecond));
    SmartDashboard.putNumber(
        "Elevator Temperature Left (Celsius)",
        leadElevatorMotor.getDeviceTemp().getValue().in(Celsius));
    SmartDashboard.putNumber(
        "Elevator Temperature Right (Celsius)",
        rightElevatorMotor.getDeviceTemp().getValue().in(Celsius));
    SmartDashboard.putNumber(
        "Elevator Temperature Top (Celsius)",
        topElevatorMotor.getDeviceTemp().getValue().in(Celsius));
    SmartDashboard.putNumber(
        "Elevator Acceleration (RotationsPerSecondPerSecond)",
        leadElevatorMotor.getAcceleration().getValue().in(RotationsPerSecondPerSecond));
  }

  /**
   * Gets the left elevator motor instance.
   *
   * @return The left TalonFX motor instance.
   */
  public TalonFX getLeftMotor() {
    return leadElevatorMotor;
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
   * This method verifies several safety conditions, including velocity,
   * acceleration,
   * temperature, and position limits. If any of these conditions are violated,
   * the subsystem is
   * disabled (motors are set to brake mode), and an appropriate alert is raised.
   * If all conditions
   * are safe, the subsystem remains operational.
   *
   * @return {@code true} if the elevator passes all safety checks and is
   *         operating safely, {@code
   *     false} otherwise.
   */
  private boolean safetyCheck() {
    if (!RobotConstants.runSafetyCheck)
      return true;

    AngularVelocity maxAngularVelocity = RotationsPerSecond.of(
        ElevatorConstants.UNSAFE_SPEED.in(MetersPerSecond)
            / ElevatorConstants.kMetersPerRotation);

    AngularAcceleration maxAngularAcceleration = RotationsPerSecondPerSecond.of(
        ElevatorConstants.UNSAFE_ACCELERATION.in(MetersPerSecondPerSecond)
            / ElevatorConstants.kMetersPerRotation);

    if (leadElevatorMotor.getVelocity().getValue().abs(RotationsPerSecond) >= maxAngularVelocity
        .in(RotationsPerSecond)) {
      brake();
      velocityAlert.set(true);
      return false;
    } else
      velocityAlert.set(false);

    if (leadElevatorMotor.getAcceleration().getValue().abs(RotationsPerSecondPerSecond) >= maxAngularAcceleration
        .in(RotationsPerSecondPerSecond)) {
      brake();
      accelerationAlert.set(true);
      return false;
    } else
      accelerationAlert.set(false);

    if (leadElevatorMotor.getDeviceTemp().getValue().gte(ElevatorConstants.MAX_TEMPERATURE)
        || rightElevatorMotor.getDeviceTemp().getValue().gte(ElevatorConstants.MAX_TEMPERATURE)
        || topElevatorMotor.getDeviceTemp().getValue().gte(ElevatorConstants.MAX_TEMPERATURE)) {
      brake();
      overheatingAlert.set(true);
      return false;
    } else
      overheatingAlert.set(false);

    if ((getHeight()
        .gt(ElevatorConstants.kMaxHeight.plus(ElevatorConstants.OVEREXTENSION_TOLERANCE))
        || getHeight()
            .lt(ElevatorConstants.kMinHeight.minus(ElevatorConstants.OVEREXTENSION_TOLERANCE)))) {
      brake();
      positionAlert.set(true);
      return false;
    } else {
      coast();
      positionAlert.set(false);
    }
    return true;
  }
}
