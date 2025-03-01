package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.util.CombinedAlert;

/**
 * The {@code WristSubsystem} class controls the wrist subsystem of the robot. It manages the motion
 * and angle of the elevator using TalonFX motors with a fused CANCoder.
 */
public class WristSubsystem extends SubsystemBase {
  private final TalonFX wristMotor = new TalonFX(WristConstants.kWristMotorID, "rio");
  private final DutyCycleEncoder wristEncoder =
      new DutyCycleEncoder(WristConstants.kWristEncoderID);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final ArmFeedforward wristFeedforward;
  // private final MotionMagicExpoVoltage motionMagicRequest =
  //     new MotionMagicExpoVoltage(0).withSlot(0);

  private final PIDController pidController;

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
    wristFeedforward =
        new ArmFeedforward(
            WristConstants.feedforwardKs,
            WristConstants.feedforwardKg,
            WristConstants.feedforwardKv);
    pidController =
        new PIDController(WristConstants.pidKp, WristConstants.pidKi, WristConstants.pidKd);
    brake();
  }

  /**
   * Sets the wrist power in manual mode.
   *
   * @param power The power to set, ranging from -1 to 1. Positive values move the elevator up, and
   *     negative values move it down.
   */
  public void setPower(double power) {
    wristMotor.set(power);
    /*
    wristMotor.setControl(
        velocityRequest
            .withVelocity(power * WristConstants.kMaxSpeed.in(RotationsPerSecond))
            .withLimitForwardMotion(getPosition().gte(WristConstants.kMaxAngle))
            .withLimitReverseMotion(getPosition().lte(WristConstants.kMinAngle)));
             */
  }

  private double calculateFeedForward() {
    return wristFeedforward.calculate(getPosition().in(Radians), 0);
  }

  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
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
    return Rotations.of(wristEncoder.get());
  }

  public AngularVelocity getVelocity() {
    return wristMotor.getVelocity().getValue();
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Angle (Rotations)", getPosition().in(Rotations));
    SmartDashboard.putNumber("Wrist Angle (Degrees)", getPosition().in(Degrees));
    SmartDashboard.putNumber(
        "Wrist Velocity (RotationsPerSecond)", getVelocity().in(RotationsPerSecond));
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
  private boolean safetyCheck() {
    if (!RobotConstants.runSafetyCheck) return true;

    if (getVelocity().abs(RotationsPerSecond)
        >= WristConstants.UNSAFE_SPEED.in(RotationsPerSecond)) {
      brake();
      velocityAlert.set(true);
      return false;
    } else velocityAlert.set(false);

    if (wristMotor.getDeviceTemp().getValue().gte(WristConstants.MAX_TEMPERATURE)) {
      brake();
      overheatingAlert.set(true);
      return false;
    } else {
      overheatingAlert.set(false);
    }

    Angle currentAngle = getPosition();
    if (currentAngle.gt(WristConstants.kMaxAngle.plus(WristConstants.kAngleTolerance))
        || currentAngle.lt(WristConstants.kMinAngle.minus(WristConstants.kAngleTolerance))) {
      brake();
      positionAlert.set(true);
      return false;
    } else {
      positionAlert.set(false);
      brake();
    }
    return true;
  }
}
