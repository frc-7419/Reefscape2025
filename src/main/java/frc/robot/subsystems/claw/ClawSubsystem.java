package frc.robot.subsystems.claw;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ClawConstants;
import frc.robot.util.CombinedAlert;

public class ClawSubsystem extends SubsystemBase {

  private TalonFX clawMotor;
  private CANcoder absEncoder;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private final CombinedAlert velocityAlert =
      new CombinedAlert(
          CombinedAlert.Severity
              .WARNING, // im assuming velocity being high isnt THAT big of a problem so just
          // warning
          "Algae claw error",
          "The claw velocity is too high. Speed is now limited.");

  private enum ControlMode {
    MANUAL,
    MOTIONMAGIC
  }

  private ControlMode controlMode = ControlMode.MANUAL;

  public ClawSubsystem() {
    clawMotor = new TalonFX(Constants.ClawConstants.kClawMotorId);
    absEncoder = new CANcoder(Constants.ClawConstants.kAbsoluteEncoderChannel);
    clawMotor.getConfigurator().apply(Constants.ClawConstants.kMotionMagicConfig);
  }

  private boolean safetyCheck() {
    double currentVelocityRPS = Math.abs(absEncoder.getVelocity().getValueAsDouble());
    if (currentVelocityRPS > ClawConstants.kMaxSafeVelocityRPS) {
      velocityAlert.set(true);
      clawMotor.setControl(new DutyCycleOut(0.0));

      return false;
    } else {
      velocityAlert.set(false);
      return true;
    }
  }

  public void setPower(double power) {
    if (controlMode == ControlMode.MOTIONMAGIC || !safetyCheck()) {
      return;
    }
    power = Math.max(-1.0, Math.min(1.0, power));
    double targetRPS = power * ClawConstants.kMaxSafeVelocityRPS;

    clawMotor.setControl(velocityRequest.withVelocity(targetRPS));
  }

  public void setClosingVoltage(double speed) {
    if (safetyCheck()) {
      clawMotor.setControl(new DutyCycleOut(speed));
    }
  }

  public void setOpeningVoltage(double speed) {
    if (safetyCheck()) {
      clawMotor.setControl(new DutyCycleOut(-speed));
    }
  }

  public void brake() {
    clawMotor.setControl(new DutyCycleOut(0.0));
    clawMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void coast() {
    clawMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public Angle getPositionDegrees() {
    return absEncoder.getPosition().getValue();
  }

  public double getPositionDouble() {
    return clawMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    double currentVelocity = absEncoder.getVelocity().getValueAsDouble();
    SmartDashboard.putNumber("Claw Velocity RPS", currentVelocity);
  }
}
