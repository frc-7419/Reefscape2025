package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.MetersPerSecond;
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
import frc.robot.util.CombinedAlert;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  private TalonFX clawMotor;
  private CANcoder absEncoder;
  private DigitalInput beamBreak;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  public AlgaeIntakeSubsystem() {
    this.clawMotor = new TalonFX(ClawConstants.kClawMotorId);
    this.absEncoder = new CANcoder(ClawConstants.kAbsoluteEncoderChannel);
    this.beamBreak = new DigitalInput(ClawConstants.kBeambreakid);
    clawMotor.getConfigurator().apply(ClawConstants.kMotionMagicConfig);
  }

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

  private enum ControlMode {
    MANUAL,
    MOTIONMAGIC
  }

  private ControlMode controlMode = ControlMode.MANUAL;

  /**
   * Set the power to the motor using MotionMagics
   *
   * @param power
   */
  public void setPower(double power) {
    if (controlMode == ControlMode.MOTIONMAGIC || !safetyCheck()) return;
    power = Math.max(-1, Math.min(1, power));
    clawMotor.setVoltage(power * 12);
  }

  /**
   * Get the status of the beam break whether it has been broken or not
   *
   * @return true if the beam has been broken, false if otherwise
   */
  public Boolean getBeamBreak() {
    return beamBreak.get();
  }

  /**
   * Set the algae intake to a specific angle using MotionMagic
   *
   * @param angle
   */
  private void toPosition(Angle angle) {
    controlMode = ControlMode.MOTIONMAGIC;
    if (!safetyCheck()) return;
    clawMotor.setControl(positionRequest.withPosition(angle));
  }

  /**
   * Change the ControlMode of the algae intake motor
   *
   * @param control
   */
  private void switchControlMode(ControlMode control) {
    controlMode = control;
  }

  /**
   * Set the angle of the algae intake with the manual control mode
   *
   * @param angle
   * @return a command moving the algae intake to the desired angle using the manual control mode
   */
  public Command setPosition(Angle angle) {
    return this.runEnd(() -> toPosition(angle), () -> switchControlMode(ControlMode.MANUAL));
  }

  /**
   * Set the speed of the claw motor to a specific angular velocity
   *
   * @param speed
   */
  public void setSpeed(AngularVelocity speed) {
    clawMotor.setControl(velocityRequest.withVelocity(speed));
  }

  /** Set the control mode of the motor to coast */
  public void coast() {
    clawMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setClosingVoltage(double voltage) {
    clawMotor.setVoltage(voltage);
  }

  public void setOpeningVoltage(double voltage) {
    clawMotor.setVoltage(-voltage);
  }

  /** Set the control mode of the motor to brake */
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

  public double getVelocityAsDouble() {
    return clawMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw Velocity", absEncoder.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Passing Safety Checks", safetyCheck());
  }

  /**
   * Performs a safety check on the claw motor.
   *
   * <p>This method checks if the claw motor's velocity and temperature are within safe limits. If
   * the velocity exceeds the maximum allowed angular velocity or the temperature exceeds the
   * maximum allowed temperature, the motor is braked, and an alert is set.
   *
   * @return true if all safety checks pass, false otherwise.
   */
  private boolean safetyCheck() {
    AngularVelocity maxAngularVelocity =
        RotationsPerSecond.of(
            ClawConstants.UNSAFE_SPEED.in(MetersPerSecond) / ClawConstants.kMetersPerRotation);
    if (!Constants.RobotConstants.runSafetyCheck) return true;
    if (getVelocity().abs(RotationsPerSecond) >= maxAngularVelocity.in(RotationsPerSecond)) {
      brake();
      velocityAlert.set(true);
      return false;
    } else {
      velocityAlert.set(false);
    }
    if (clawMotor.getDeviceTemp().getValue().gte(ClawConstants.MAX_TEMPERATURE)) {
      brake();
      overheatingAlert.set(true);
      return false;
    } else {
      overheatingAlert.set(false);
    }
    return true;
  }
}
