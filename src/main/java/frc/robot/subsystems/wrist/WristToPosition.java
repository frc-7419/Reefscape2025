package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.WristConstants;

public class WristToPosition extends Command {
  WristSubsystem wristSubsystem;
  Angle setpoint;

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
    pidController.setTolerance(0.1);
  }

  @Override
  public void execute() {
    double currentPos = wristSubsystem.getPosition().in(Rotations);
    double pidOutput = pidController.calculate(currentPos);
    pidOutput = Math.max(-5, Math.min(pidOutput, 5));

    wristSubsystem.setVoltage(pidOutput);
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
