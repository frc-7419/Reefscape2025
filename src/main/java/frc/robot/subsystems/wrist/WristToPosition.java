// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.WristConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristToPosition extends Command {
  /** Creates a new WristToPosition. */
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
  }

  @Override
  public void execute() {
    double currentPos = wristSubsystem.getPosition().in(Rotations);
    double pidOutput = pidController.calculate(currentPos);
    pidOutput = MathUtil.clamp(pidOutput, -5, 5);

    wristSubsystem.setVoltage(pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
