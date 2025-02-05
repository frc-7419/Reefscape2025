// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunClawWithBeamBreak extends Command {
  /** Creates a new OpenClaw. */
  private ClawSubsystem clawSubsystem;

  public RunClawWithBeamBreak(ClawSubsystem clawSubsystem) {
    this.clawSubsystem = clawSubsystem;
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (clawSubsystem.getBeamBreak()) {
      // if algae is there, apply a small power so it doesn't fall off. tested with hardware, algae
      // doesn't stay there by itself
      clawSubsystem.setPower(0.1);
    }
    if (!(clawSubsystem.getBeamBreak())) {
      clawSubsystem.setPower(0.6);
    }
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
