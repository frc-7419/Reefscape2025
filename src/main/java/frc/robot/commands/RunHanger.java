// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunHanger extends Command {
  /** Creates a new RunHanger. */
  private ElevatorSubsystem elevatorSubsystem;

  private boolean humanInterference = false; // Human hanging or auto hanging
  private CommandXboxController xboxController;

  public RunHanger(
      ElevatorSubsystem elevatorSubsystem,
      boolean humanInterference,
      CommandXboxController xboxController) {
    this.xboxController = xboxController;
    this.humanInterference = humanInterference;
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (humanInterference) {
      elevatorSubsystem.setPower(xboxController.getLeftY()); // Arbitrary joystick input
    } else {
      elevatorSubsystem.setPower(-0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
