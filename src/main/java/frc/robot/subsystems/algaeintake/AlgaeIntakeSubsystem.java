package frc.robot.subsystems.algaeintake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.AlgaeIntakeConstants;
import frc.robot.util.CombinedAlert;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  private TalonFX clawMotor;
  private DigitalInput beamBreak;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public AlgaeIntakeSubsystem() {
    this.clawMotor = new TalonFX(AlgaeIntakeConstants.kClawMotorId);
    this.beamBreak = new DigitalInput(AlgaeIntakeConstants.kBeambreakid);
    clawMotor.getConfigurator().apply(AlgaeIntakeConstants.kMotionMagicConfig);
  }

  private final CombinedAlert velocityAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Algae Intake Velocity Error",
          "The Algae Intake velocity is outside the safe range. Subsystem disabled.");

  private final CombinedAlert overheatingAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Algae Intake Overheating",
          "The Algae Intake motor is overheating. Subsystem disabled.");

  private enum ControlMode {
    MANUAL,
    MOTIONMAGIC
  }

  private ControlMode controlMode = ControlMode.MANUAL;

  public void setPower(double power) {
    if (controlMode == ControlMode.MOTIONMAGIC || !safetyCheck()) return;
    power = Math.max(-1, Math.min(1, power));
    clawMotor.setVoltage(power * 12);
  }

  public boolean getBeamBreak() { // primative boolean instead of Boolean
    return beamBreak.get();
  }

  private void switchControlMode(ControlMode control) {
    controlMode = control;
  }

  public void setSpeed(AngularVelocity speed) {
    clawMotor.setControl(velocityRequest.withVelocity(speed));
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
    clawMotor.setVoltage(0);
    clawMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public AngularVelocity getVelocity() {
    return clawMotor.getVelocity().getValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Passing Safety Checks", safetyCheck());
  }

  private boolean safetyCheck() {
    AngularVelocity maxAngularVelocity =
        RotationsPerSecond.of(
            AlgaeIntakeConstants.UNSAFE_SPEED.in(MetersPerSecond)
                / AlgaeIntakeConstants.kMetersPerRotation);
    if (!Constants.RobotConstants.runSafetyCheck) return true;
    if (getVelocity().abs(RotationsPerSecond) >= maxAngularVelocity.in(RotationsPerSecond)) {
      brake();
      velocityAlert.set(true);
      return false;
    } else {
      velocityAlert.set(false);
    }
    if (clawMotor.getDeviceTemp().getValue().gte(AlgaeIntakeConstants.MAX_TEMPERATURE)) {
      brake();
      overheatingAlert.set(true);
      return false;
    } else {
      overheatingAlert.set(false);
    }
    return true;
  }
}
