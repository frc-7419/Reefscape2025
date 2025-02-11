// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.groundintake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.util.CombinedAlert;

public class GroundIntakeSubsystem extends SubsystemBase {
  /** Creates a new GroundIntakeSubsystem. */
  private final TalonFX groundIntakeMotor;

  public GroundIntakeSubsystem() {
    this.groundIntakeMotor =
        new TalonFX(Constants.GroundIntakeIntakeConstants.kGroundIntakeIntakeMotorID);
  }

  private final CombinedAlert temperatureAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Ground Intake Temperature Alert",
          "The ground intake end effector  temperature is too high. Subsystem disabled.");
  private final CombinedAlert velocityAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "Ground Intake Velocity Alert",
          "The ground intake end effector velocity is too high. Subsystem disabled.");

  public void setVoltage(double power) {
    if (!safetyCheck()) {
      return;
    }
    groundIntakeMotor.setVoltage(power);
  }

  public void setSpeed(double speed) {
    if (!safetyCheck()) {
      return;
    }
    groundIntakeMotor.set(speed);
  }

  public void brake() {
    groundIntakeMotor.setVoltage(0);
    groundIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void coast() {
    groundIntakeMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public Temperature getTemperature() {
    return groundIntakeMotor.getDeviceTemp().getValue();
  }

  public AngularVelocity getVelocity() {
    return groundIntakeMotor.getVelocity().getValue();
  }

  private boolean safetyCheck() {
    if (!RobotConstants.runSafetyCheck) {
      return true;
    }
    if (getVelocity().abs(RotationsPerSecond)
        >= Constants.GroundIntakeIntakeConstants.UNSAFE_SPEED.in(RotationsPerSecond)) {
      brake();
      velocityAlert.set(true);
      return false;
    } else {
      velocityAlert.set(false);
    }
    if (getTemperature().gte(Constants.GroundIntakeIntakeConstants.MAX_TEMPERATURE)) {
      brake();
      temperatureAlert.set(true);
      return false;
    } else {
      temperatureAlert.set(false);
    }
    return true;
  }

  @Override
  public void periodic() {}
}
