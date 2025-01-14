// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToSetpoint extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final ProfiledPIDController pidController;
  private final double pos;
  private final TrapezoidProfile.Constraints constraints;

  /** Creates a new ElevatorToSetpoint. */
  public ElevatorToSetpoint(
      ElevatorSubsystem elevatorSubsystem,
      double pos,
      double kp,
      double kd,
      TrapezoidProfile.Constraints constraints) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.pos = pos;
    this.constraints = constraints;
    pidController = new ProfiledPIDController(kp, kd, 0, constraints);
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setGoal(pos);
    pidController.setTolerance(1000); // placeholder
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
