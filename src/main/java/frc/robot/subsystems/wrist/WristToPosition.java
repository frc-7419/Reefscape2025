package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.WristConstants;

public class WristToPosition extends Command {
  WristSubsystem wristSubsystem;
  Angle setpoint;
  ArmFeedforward ff = new ArmFeedforward(0, 0.2, 0);

  private PIDController pidController =
      new PIDController(WristConstants.pidKp, WristConstants.pidKi, WristConstants.pidKd);

  public WristToPosition(WristSubsystem wristSubsystem, Angle setpoint) {
    this.wristSubsystem = wristSubsystem;
    this.setpoint = setpoint;
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(setpoint.in(Rotations));
    pidController.setTolerance(0.02);
  }

  @Override
  public void execute() {
    double currentPos = wristSubsystem.getPosition().in(Rotations);
    double pidOutput = pidController.calculate(currentPos);
    pidOutput = Math.max(-5, Math.min(pidOutput, 5));
    double feedforwardCalculation =
        ff.calculate(wristSubsystem.getPosition().minus(Degrees.of(90)).in(Radians), 0);
    pidOutput += feedforwardCalculation;

    wristSubsystem.setVoltage(pidOutput);
    SmartDashboard.putNumber("Wrist Error", setpoint.in(Rotations) - currentPos);
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    // return pidController.atSetpoint();
    return false;
  }
}
