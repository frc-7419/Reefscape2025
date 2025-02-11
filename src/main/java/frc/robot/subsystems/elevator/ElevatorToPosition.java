// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.Meters;
import frc.robot.constants.Constants.ElevatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToPosition extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  private Distance position;
  private final boolean IS_TUNING = true;
  
  /** Creates a new ElevatorToPosition. */
  public ElevatorToPosition(ElevatorSubsystem elevatorSubsystem, Distance position) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.position = position;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.coast();
    elevatorSubsystem.setGoal(position.in(Meters) / ElevatorConstants.kMetersPerRotation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
    elevatorSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // check if position is equal
    return false;
  }
}
