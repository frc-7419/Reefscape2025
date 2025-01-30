// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunGroundIntake extends Command {
  private final GroundIntakeSubsystem groundIntakeSubsystem;
  /** Creates a new RunGroundIntake. */
  public RunGroundIntake(GroundIntakeSubsystem groundIntakeSubsystem) {
    this.groundIntakeSubsystem = groundIntakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(groundIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    groundIntakeSubsystem.coast();
    groundIntakeSubsystem.setSpeed(0.1); // Arbitrary number
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (groundIntakeSubsystem.beamBreakIsTriggered()) {
      groundIntakeSubsystem.setSpeed(0);
      groundIntakeSubsystem.brake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    groundIntakeSubsystem.setSpeed(0);
    groundIntakeSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return groundIntakeSubsystem.beamBreakIsTriggered();
  }
}
