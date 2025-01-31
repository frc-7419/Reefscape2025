// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClawConstants;
import frc.robot.subsystems.claw.ClawSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CloseClaw extends Command {
  /** Creates a new OpenClaw. */
  private ClawSubsystem clawSubsystem;

  private Angle setpoint;

  public CloseClaw(
      ClawSubsystem clawSubsystem,
      Angle setpoint) { // TODO: figure out the value of setpoint for the desired claw open
    this.clawSubsystem = clawSubsystem;
    this.setpoint = setpoint;
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    clawSubsystem.coast();
  }

  @Override
  public void execute() {
    clawSubsystem.setPosition(ClawConstants.kClawCloseSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
