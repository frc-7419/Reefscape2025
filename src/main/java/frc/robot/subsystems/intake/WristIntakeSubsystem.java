// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.constants.Constants.WristIntakeConstants;
import frc.robot.util.CombinedAlert;

public class WristIntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private static final double CURRENT_THRESHOLD = 100; // needs to be checked with tuning
  private double baselineCurrentDraw;
  private DigitalInput beamBreak = new DigitalInput(2);

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

  /** Creates a new WristIntakeSubsystem. */
  public WristIntakeSubsystem() {
    this.intakeMotor = new TalonFX(WristIntakeConstants.kWristIntakeMotorID, "rio");
    intakeMotor.getConfigurator().apply(WristIntakeConstants.kWristIntakeTalonFXConfiguration);
    baselineCurrentDraw = intakeMotor.getMotorVoltage().getValueAsDouble();
  }

  public void coast() {
    intakeMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void brake() {
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setVoltage(double voltage) {
    intakeMotor.set(voltage);
  }

  public void setPower(double power) {
    if (!safetyCheck()) return;
    power = Math.max(-1, Math.min(1, power));
    intakeMotor.set(power);
  }

  public boolean beamBreakisTriggered() {
    return beamBreak.get();
  }
  /**
   * Returns a Command that applies manual wrist control (ex, from a joystick).
   *
   * @param power The power from -1 to 1.
   * @return A command for scheduling.
   */
  public Command joystickControl(double power) {
    return this.run(() -> setPower(power));
  }

  public Voltage getVoltage() {
    return intakeMotor.getMotorVoltage().getValue();
  }

  public Current getCurrent() {
    return intakeMotor.getTorqueCurrent().getValue();
  }

  public AngularVelocity getVelocity() {
    return intakeMotor.getVelocity().getValue();
  }

  public boolean coralDetectedByCurrent() {
    double currentDraw = getCurrent().in(Amps);
    return Math.abs(currentDraw) > CURRENT_THRESHOLD;
  }

  public void updateBaselineCurrentDraw() {
    baselineCurrentDraw = getCurrent().in(Amps);
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
        >= WristIntakeConstants.UNSAFE_SPEED.in(RotationsPerSecond)) {
      brake();
      velocityAlert.set(true);
      return false;
    } else velocityAlert.set(false);
    if (intakeMotor.getDeviceTemp().getValue().gte(WristIntakeConstants.MAX_TEMPERATURE)) {
      brake();
      overheatingAlert.set(true);
      return false;
    } else overheatingAlert.set(false);
    return true;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Coral Speed: ", intakeMotor.get());
    SmartDashboard.putNumber("Wrist Intake Voltage", getVoltage().in(Volts));
    SmartDashboard.putNumber("Wrist Intake Current", getCurrent().in(Amps));
    SmartDashboard.putBoolean("Beam Break Triggred", beamBreakisTriggered());
  }
}
