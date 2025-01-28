// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.WristIntakeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreL4 extends Command {
  /** Creates a new ScoreL4. */
  private final ElevatorSubsystem elevator;

  private final WristSubsystem wrist;
  private final WristIntakeSubsystem intake;
  private final double elevatorSetPoint;
  private final double wristSetPoint;

  public ScoreL4(WristSubsystem wrist, ElevatorSubsystem elevator, WristIntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.elevator = elevator;
    this.intake = intake;
    this.elevatorSetPoint = Constants.ScoringL4Constants.elevatorSetPoint;
    this.wristSetPoint = Constants.ScoringL4Constants.wristSetPoint;
    addRequirements(wrist, elevator, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.coast();
    elevator.coast();
    intake.coastCoral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // Need the constants as a distance
    Distance distance1 = Inches.of(Constants.ScoringL4Constants.elevatorSetPoint);
    elevator.setPosition(distance1); // Random Value

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.brake();
    elevator.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
