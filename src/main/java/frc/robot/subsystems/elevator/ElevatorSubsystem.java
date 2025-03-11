package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.util.CombinedAlert;
import java.util.function.Supplier;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX leftElevatorMotor =
      new TalonFX(ElevatorConstants.kLeftElevatorMotorId, RobotConstants.kCANivoreBus);
  private final TalonFX rightElevatorMotor =
      new TalonFX(ElevatorConstants.kRightElevatorMotorId, RobotConstants.kCANivoreBus);
  private final TalonFX topElevatorMotor =
      new TalonFX(ElevatorConstants.kTopElevatorMotorId, RobotConstants.kCANivoreBus);

  private final Supplier<Angle> wristAngleSupplier;
  private final Supplier<Pigeon2> robotPigeonSupplier;
  private boolean tipLock = false;

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.feedforwardKs,
          ElevatorConstants.feedforwardKg,
          ElevatorConstants.feedforwardKv,
          ElevatorConstants.feedforwardKa);

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final SysIdRoutine routine =
      new SysIdRoutine(
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

  private final CombinedAlert positionAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Elevator Out of Range",
          "The encoder value is outside the safe range. Subsystem disabled.");

  private final CombinedAlert velocityAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Elevator Velocity Error",
          "The elevator velocity is outside the safe range. Subsystem disabled.");

  private final CombinedAlert accelerationAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Elevator Acceleration Error",
          "The elevator acceleration is outside the safe range. Subsystem disabled.");

  private final CombinedAlert overheatingAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Elevator Overheating",
          "The elevator motors are overheating. Subsystem disabled.");

  /** Creates a new {@code ElevatorSubsystem}. */
  public ElevatorSubsystem(
      Supplier<Angle> wristAngleSupplier, Supplier<Pigeon2> robotPigeonSupplier) {
    this.wristAngleSupplier = wristAngleSupplier;
    this.robotPigeonSupplier = robotPigeonSupplier;

    leftElevatorMotor.getConfigurator().apply(ElevatorConstants.kElevatorTalonFXConfiguration);
    rightElevatorMotor.getConfigurator().apply(ElevatorConstants.kElevatorTalonFXConfiguration);
    topElevatorMotor.getConfigurator().apply(ElevatorConstants.kElevatorTalonFXConfiguration);

    brake();
    zeroEncoder();
  }

  /**
   * Sets the elevator power in manual mode.
   *
   * @param power The power to set, ranging from -1 to 1. Positive values move the elevator up, and
   *     negative values move it down.
   */
  public void setPower(double power) {
    leftElevatorMotor.setVoltage(power * 12);
    rightElevatorMotor.setVoltage(power * 12);
    topElevatorMotor.setVoltage(power * 12);
  }

  public void setVelocity(double velocity) {
    double voltage = feedforward.calculate(velocity);
    setVoltage(voltage);
  }

  public void setVoltage(double voltage) {
    if (!checkMovementSafe(voltage) && voltage != 0) {
      voltage = feedforward.calculate(0);
    }

    if (tipLock) return;

    leftElevatorMotor.setVoltage(voltage);
    rightElevatorMotor.setVoltage(voltage);
    topElevatorMotor.setVoltage(voltage);
  }

  /**
   * Moves the elevator to the specified position using Motion Magic.
   *
   * @param position The desired position as an Angle.
   */
  public void positionMM(Angle setpoint) {

    if (!safetyCheck()) return;

    if (!checkMovementSafe(setpoint.minus(getPosition()).in(Rotations))) {
      double voltage = feedforward.calculate(0);
      setVoltage(voltage);
      return;
    }

    leftElevatorMotor.setControl(
        motionMagicRequest
            .withPosition(setpoint)
            .withLimitForwardMotion(getPosition().gt(ElevatorConstants.kMaxRotations))
            .withLimitReverseMotion(getPosition().lt(ElevatorConstants.kMinRotations)));

    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), false));
    topElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), false));
  }

  /**
   * Sets the motors to coast mode.
   *
   * <p>In coast mode, the motors will spin freely when no power is applied.
   */
  public void coast() {
    leftElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    topElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void brake() {
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
    return true;
    /*
    boolean aboveLimit = getPosition().gt(ElevatorConstants.kElevatorBarUpperLimit);
    boolean belowLimit = getPosition().lt(ElevatorConstants.kElevatorBarLowerLimit);
    boolean wristUnsafe = wristAngleSupplier.get().gt(WristConstants.kElevatorSafeWristAngle);

    if (wristUnsafe && ((belowLimit && voltage > 0) || (aboveLimit && voltage < 0))) {
      return false;
    }

    return true;
    */
  }

  public Current getCurrent() {
    return leftElevatorMotor.getStatorCurrent().getValue();
  }

  public AngularVelocity getVelocity() {
    return leftElevatorMotor.getVelocity().getValue();
  }

  public boolean isStalling() {
    return getVelocity().isNear(RotationsPerSecond.of(0), RotationsPerSecond.of(0.1))
        && (getCurrent().abs(Amps) > 50);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder Position", getPosition().in(Rotations));
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

    SmartDashboard.putNumber("Elevator Current", getCurrent().in(Amps));
    SmartDashboard.putBoolean("Is Elevator Stalling", isStalling());

    SmartDashboard.putBoolean("TIPPING", tipLock);

    double pitch = robotPigeonSupplier.get().getPitch().getValue().in(Degrees);
    double roll = robotPigeonSupplier.get().getRoll().getValue().in(Degrees);

    if (isTipping(pitch, roll, RobotConstants.kTippingThresholdDeg)) {
      tipLock = true;
      positionMM(Rotations.of(0));
    } else {
      tipLock = false;
    }
  }

  private boolean isTipping(double pitchDegrees, double rollDegrees, double thresholdDeg) {
    return (Math.abs(pitchDegrees) > thresholdDeg) || (Math.abs(rollDegrees) > thresholdDeg);
  }

  /**
   * Performs a safety check to ensure the elevator operates within safe parameters.
   *
   * <p>This method verifies several safety conditions, including temperature and position limits.
   * If any of these conditions are violated, the subsystem is disabled (motors are set to brake
   * mode), and an appropriate alert is raised. If all conditions are safe, the subsystem remains
   * operational.
   *
   * @return {@code true} if the elevator passes all safety checks and is operating safely, {@code
   *     false} otherwise.
   */
  private boolean safetyCheck() {
    if (!RobotConstants.runSafetyCheck) return true;

    if (leftElevatorMotor.getDeviceTemp().getValue().gte(ElevatorConstants.MAX_TEMPERATURE)
        || rightElevatorMotor.getDeviceTemp().getValue().gte(ElevatorConstants.MAX_TEMPERATURE)
        || topElevatorMotor.getDeviceTemp().getValue().gte(ElevatorConstants.MAX_TEMPERATURE)) {
      brake();
      overheatingAlert.set(true);
      return false;
    } else {
      overheatingAlert.set(false);
    }

    if (getPosition()
            .gt(ElevatorConstants.kMaxRotations.plus(ElevatorConstants.OVEREXTENSION_TOLERANCE))
        || getPosition()
            .lt(ElevatorConstants.kMinRotations.minus(ElevatorConstants.OVEREXTENSION_TOLERANCE))) {
      positionAlert.set(true);
      return false;
    } else {
      positionAlert.set(false);
    }
    return true;
  }
}
