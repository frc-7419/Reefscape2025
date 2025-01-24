// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CloseClaw extends Command {
  /** Creates a new OpenClaw. */
  private ClawSubsystem clawSubsystem;

  private PIDController pid;

  public CloseClaw(
      ClawSubsystem clawSubsystem,
      double setpoint) { // TODO: figure out the value of setpoint for the desired claw open
    this.clawSubsystem = clawSubsystem;
    this.pid = pid;
    pid.setPID(0.5, 0, 0.1); // Need to be tuned
    pid.setSetpoint(setpoint);
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawSubsystem.setClosingVoltage(1.00);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidPower = pid.calculate(clawSubsystem.getPose());
    clawSubsystem.setOpeningVoltage(pidPower);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
