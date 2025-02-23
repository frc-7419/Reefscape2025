// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.TunableValue;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevatorWithPID extends Command {
  private final ElevatorSubsystem elevator;
  private final PIDController pidController;
  private final ElevatorFeedforward feedforward;
  private final double kP = 0.59;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kS = 0.52;
  private final double kG = 0.42;
  private final double kV = 0;
  private final double kA = 0;
  private final double setpoint;

  /** Creates a new RunElevatorWithPID. */
  public RunElevatorWithPID(ElevatorSubsystem elevator, double setpoint) {
    // WE NEED TO SET THE LINEAR CONVERSION RATES AND STUFF. THATS WHY I HAVE IT AS
    // DOUBLE FOR NOW..
    this.elevator = elevator;
    this.setpoint = setpoint;
    pidController =
        new PIDController(
            kP, kI, kD);
    feedforward =
        new ElevatorFeedforward(kS, kG, kV, kA);
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(setpoint);
    pidController.setTolerance(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidCalculation = pidController.calculate(elevator.getPosition().in(Rotations));
    pidCalculation = Math.max(-2, Math.min(pidCalculation, 5));

    double feedforwardCalculation = feedforward.calculate(0);

    elevator.setVoltage(pidCalculation + feedforwardCalculation);

    SmartDashboard.putNumber("PID Output", pidCalculation);
    SmartDashboard.putNumber("Feedforward Output", feedforwardCalculation);
    SmartDashboard.putBoolean("Elevator At Setpoint?", pidController.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setVoltage(0);
    elevator.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
