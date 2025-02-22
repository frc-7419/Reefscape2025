// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.constants.Constants.WristIntakeConstants;

public class WristIntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final DigitalInput beamBreak;

  /** Creates a new WristIntakeSubsystem. */
  public WristIntakeSubsystem() {
    this.intakeMotor =
        new TalonFX(WristIntakeConstants.kWristIntakeMotorID, RobotConstants.kCANivoreBus);
    this.beamBreak = new DigitalInput(0);
    intakeMotor.getConfigurator().apply(WristIntakeConstants.kWristIntakeTalonFXConfiguration);
  }

  public void coast() {
    intakeMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void brake() {
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public boolean beamBreakisTriggered() {
    return beamBreak.get();
  }

  public void setPower(double power) {
    intakeMotor.set(power);
  }

  /**
   * Returns a Command that applies manual wrist control (ex, from a joystick).
   *
   * @param power The power from -1 to 1.
   * @return A command for scheduling.
   */
  public Command joystickControl(CommandXboxController commandXboxController) {
    return this.run(
        () -> {
          double power =
              commandXboxController.getRightTriggerAxis()
                  - commandXboxController.getLeftTriggerAxis();
          if (Math.abs(power) > 0.05) {
            setPower(power);
          } else {
            setPower(0);
          }
        });
  }

  public AngularVelocity getVelocity() {
    return intakeMotor.getVelocity().getValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Coral Speed: ", intakeMotor.get());
  }
}
