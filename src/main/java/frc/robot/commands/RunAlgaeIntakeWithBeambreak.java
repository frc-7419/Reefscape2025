// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunAlgaeIntakeWithBeambreak extends Command {
  /** Creates a new OpenClaw. */
  private AlgaeIntakeSubsystem algaeIntakeSubsystem;

  public RunAlgaeIntakeWithBeambreak(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;
    addRequirements(algaeIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeIntakeSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (algaeIntakeSubsystem.getBeamBreak()) {
      // if algae is there, apply a small power so it doesn't fall off. tested with hardware, algae
      // doesn't stay there by itself
      algaeIntakeSubsystem.setPower(0.1);
    }
    if (!(algaeIntakeSubsystem.getBeamBreak())) {
      algaeIntakeSubsystem.setPower(0.6);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntakeSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
