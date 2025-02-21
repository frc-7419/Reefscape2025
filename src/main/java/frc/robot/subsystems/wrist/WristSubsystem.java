package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.util.CombinedAlert;

/**
 * The {@code WristSubsystem} class controls the wrist subsystem of the robot. It manages the motion
 * and angle of the elevator using TalonFX motors with a fused CANCoder.
 */
public class WristSubsystem extends SubsystemBase {
  private final TalonFX wristMotor = new TalonFX(WristConstants.kWristMotorID, "7419");
  private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(0);


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
          "Wrist Out of Range",
          "The wrist angle is outside the safe range. Subsystem disabled.");

  private final CombinedAlert velocityAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Wrist Velocity Error",
          "The wrist velocity is outside the safe range. Subsystem disabled.");

  private final CombinedAlert overheatingAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Wrist Overheating",
          "The wrist motor is overheating. Subsystem disabled.");

  /** Creates a new {@code WristSubsystem} with a TalonFX and a CANcoder. */
  public WristSubsystem() {
    wristMotor.getConfigurator().apply(WristConstants.kWristTalonFXConfiguration);
    brake();
  }

  /**
   * Sets the wrist power in manual mode.
   *
   * @param power The power to set, ranging from -1 to 1. Positive values move the elevator up, and
   *     negative values move it down.
   */
  public void setPower(double power) {
    // if (controlMode == ControlMode.MOTIONMAGIC || !safetyCheck()) {
    //   return;
    // }

    wristMotor.setControl(new DutyCycleOut(power));

    // wristMotor.setControl(
    //     velocityRequest
    //         .withVelocity(power * WristConstants.kMaxSpeed.in(RotationsPerSecond))
    //         .withLimitForwardMotion(getPosition().gte(WristConstants.kMaxAngle))
    //         .withLimitReverseMotion(getPosition().lte(WristConstants.kMinAngle)));
  }

  /**
   * Moves the wrist to a specified angle using Motion Magic.
   *
   * @param angle The desired angle (e.g., 0° is horizontal, 90° is vertical).
   */
  private void toAngle(Angle angle) {
    controlMode = ControlMode.MOTIONMAGIC;

    // if (!safetyCheck()) {
    //   return;
    // }

    // wristMotor.setControl(
    //     motionMagicRequest
    //         .withPosition(angle.in(Rotations))
    //         .withLimitForwardMotion(getPosition().gte(WristConstants.kMaxAngle))
    //         .withLimitReverseMotion(getPosition().lte(WristConstants.kMinAngle)));
  }

  /**
   * Returns a Command that drives the wrist to a specific angle and then ends, returning the
   * control mode to MANUAL.
   *
   * @param angle The target angle
   * @return A command for scheduling.
   */
  public Command setAngle(Angle angle) {
    return this.runEnd(() -> toAngle(angle), () -> switchControlMode(ControlMode.MANUAL));
  }

  /**
   * Returns a Command that applies manual wrist control (ex, from a joystick).
   *
   * @param power The power from -1 to 1.
   * @return A command for scheduling.
   */
  public Command joystickControl(CommandXboxController joystick) {
    return this.run(() -> setPower(joystick.getRightY()));
  }

  /** Switches the current control mode. */
  private void switchControlMode(ControlMode mode) {
    this.controlMode = mode;
  }

  /** Sets the TalonFX to brake mode (resists motion when no power is applied). */
  public void brake() {
    wristMotor.set(0);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /** Sets the TalonFX to coast mode (spins freely when no power is applied). */
  public void coast() {
    wristMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public Angle getPosition() {
    double reading = wristEncoder.get();
    if (reading > 0.99) return Rotations.of(0);
    return Rotations.of(wristEncoder.get());
  }

  // public AngularVelocity getVelocity() {
  //   return wristEncoder.getVelocity().getValue();
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Angle (Rotations)", getPosition().in(Rotations));
    SmartDashboard.putNumber("Wrist Angle (Degrees)", getPosition().in(Degrees));

    SmartDashboard.putNumber(
        "Wrist Temperature (Celsius)", wristMotor.getDeviceTemp().getValue().in(Celsius));
  }

  /**
   * Performs a safety check to ensure the wrist operates within safe parameters.
   *
   * <p>This method verifies several safety conditions, including velocity, acceleration,
   * temperature, and angle limits. If any of these conditions are violated, the subsystem is
   * disabled (motors are set to brake mode), and an appropriate alert is raised. If all conditions
   * are safe, the subsystem remains operational.
   *
   * @return {@code true} if the wrist passes all safety checks and is operating safely, {@code
   *     false} otherwise.
   */
  // private boolean safetyCheck() {
  //   if (!RobotConstants.runSafetyCheck) return true;

  //   if (getVelocity().abs(RotationsPerSecond)
  //       >= WristConstants.UNSAFE_SPEED.in(RotationsPerSecond)) {
  //     brake();
  //     velocityAlert.set(true);
  //     return false;
  //   } else velocityAlert.set(false);

  //   if (wristMotor.getDeviceTemp().getValue().gte(WristConstants.MAX_TEMPERATURE)) {
  //     brake();
  //     overheatingAlert.set(true);
  //     return false;
  //   } else {
  //     overheatingAlert.set(false);
  //   }

  //   Angle currentAngle = getPosition();
  //   if (currentAngle.gt(WristConstants.kMaxAngle.plus(WristConstants.kAngleTolerance))
  //       || currentAngle.lt(WristConstants.kMinAngle.minus(WristConstants.kAngleTolerance))) {
  //     brake();
  //     positionAlert.set(true);
  //     return false;
  //   } else {
  //     positionAlert.set(false);
  //     coast();
  //   }
  //   return true;
  // }
}
