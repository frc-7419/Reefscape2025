package frc.robot.subsystems.algaepivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaePivot extends SubsystemBase {
  // TODO: add constants
  private final TalonFX pivotMotor = new TalonFX(0);
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);

  private double kS = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;
  private double kG = 0.0;
  private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);

  public AlgaePivot() {
    var talonFXConfiguration = pivotMotor.getConfigurator();
    var motorConfig = new MotorOutputConfigs();

    motorConfig.Inverted = InvertedValue.Clockwise_Positive; // check
    talonFXConfiguration.apply(motorConfig);
  }

  public Angle getAngle() {
    Angle measurement = Rotations.of(pivotEncoder.get());
    measurement = measurement.plus(Rotations.of(0.5)); // check offset
    return measurement;
  }

  public AngularVelocity getVelocity() {
    return pivotMotor.getVelocity().getValue();
  }

  public void setPower(double power) {
    pivotMotor.set(power);
  }

  public void setVelocity(double velocity) {
    double output = feedforward.calculate(getAngle().in(Radians), velocity);
    pivotMotor.setVoltage(output);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Angle", getAngle().in(Degrees));
  }
}
