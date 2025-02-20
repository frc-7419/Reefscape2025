// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.TunableValue;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPIDTest extends Command {
  private final ElevatorSubsystem elevator;
  private final PIDController pidController;
  private final ElevatorFeedforward feedforward;
  private final TunableValue kP = new TunableValue("Elevator kP", 0.0);
  private final TunableValue kI = new TunableValue("Elevator kI", 0.0);
  private final TunableValue kD = new TunableValue("Elevator kD", 0.0);
  private final TunableValue kS = new TunableValue("Elevator kS", 0.44);
  private final TunableValue kG = new TunableValue("Elevator kG", 0.32);
  private final TunableValue kV = new TunableValue("Elevator kV", 0.0);
  private final TunableValue kA = new TunableValue("Elevator kA", 0.0);
  private final TunableValue setpoint = new TunableValue("Elevator Setpoint", 0.0);

  /** Creates a new ElevatorPIDTest. */
  public ElevatorPIDTest(ElevatorSubsystem elevator) {
    // WE NEED TO SET THE LINEAR CONVERSION RATES AND STUFF. THATS WHY I HAVE IT AS
    // DOUBLE FOR NOW..
    this.elevator = elevator;
    pidController = new PIDController(kP.getValue(), kI.getValue(), kD.getValue());
    feedforward =
        new ElevatorFeedforward(kS.getValue(), kG.getValue(), kV.getValue(), kA.getValue());
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (kP.getValue() != pidController.getP()) {
      pidController.setP(kP.getValue());
    }
    if (kI.getValue() != pidController.getI()) {
      pidController.setI(kI.getValue());
    }
    if (kD.getValue() != pidController.getD()) {
      pidController.setD(kD.getValue());
    }
    if (kS.getValue() != feedforward.getKs()) {
      feedforward.setKs(kS.getValue());
    }
    if (kG.getValue() != feedforward.getKg()) {
      feedforward.setKg(kG.getValue());
    }
    if (kV.getValue() != feedforward.getKv()) {
      feedforward.setKv(kV.getValue());
    }
    if (kA.getValue() != feedforward.getKa()) {
      feedforward.setKa(kA.getValue());
    }
    if (pidController.getSetpoint() != setpoint.getValue()) {
      pidController.setSetpoint(setpoint.getValue());
    }

    elevator.setVoltage(
        pidController.calculate(elevator.getPosition().in(Rotations)) + feedforward.calculate(0));

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
