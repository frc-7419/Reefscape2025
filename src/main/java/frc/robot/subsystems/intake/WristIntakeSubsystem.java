// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.WristIntakeConstants;

public class WristIntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private static final Current CURRENT_THRESHOLD = Amps.of(50); // needs to be checked with tuning
  private DigitalInput beamBreak = new DigitalInput(2);
  Debouncer debouncer = new Debouncer(0.2, DebounceType.kBoth);

  private final TorqueCurrentFOC torqueFOC = new TorqueCurrentFOC(0);

  public boolean HOLDING = false;

  public WristIntakeSubsystem() {
    this.intakeMotor = new TalonFX(WristIntakeConstants.kWristIntakeMotorID, "rio");
    intakeMotor.getConfigurator().apply(WristIntakeConstants.kWristIntakeTalonFXConfiguration);
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
    intakeMotor.set(power);
  }

  public boolean beamBreakisTriggered() {
    return beamBreak.get();
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
    boolean thresholdReached = getCurrent().gt(CURRENT_THRESHOLD);
    SmartDashboard.putBoolean("CoralDetectedRaw", thresholdReached);
    return debouncer.calculate(thresholdReached);
  }

  public boolean getHolding() {
    return HOLDING;
  }

  public Trigger isHolding() {
    return new Trigger(this::getHolding);
  }

  public void setHolding(boolean holding) {
    this.HOLDING = holding;
  }

  public void applyControlRequest(ControlRequest request){
    intakeMotor.setControl(request);
  }

  public void setTorque(Current current){
    intakeMotor.setControl(torqueFOC.withOutput(current));
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Coral Speed: ", intakeMotor.get());
    SmartDashboard.putNumber("Wrist Intake Voltage", getVoltage().in(Volts));
    SmartDashboard.putNumber("Wrist Intake Current", getCurrent().in(Amps));
    SmartDashboard.putBoolean("Beam Break Triggred", beamBreakisTriggered());
    SmartDashboard.putBoolean("Intake Holding", HOLDING);
    if (coralDetectedByCurrent())
      HOLDING = true;
  }
}
