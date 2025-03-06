package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.ScoringConstants.ScoringSetpoint;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ScoringSetpoints extends Command {
  private final ElevatorSubsystem elevator;
  private final WristSubsystem wrist;
  private final ScoringSetpoint targetPosition;
  private final Angle upAngle;

  Angle wristSetpoint;

  private PIDController pidController =
      new PIDController(WristConstants.pidKp, WristConstants.pidKi, WristConstants.pidKd);

  private void setWristAngle(Angle targetPosition) {
    if (!targetPosition.equals(wristSetpoint)) {
      pidController.setSetpoint(targetPosition.in(Rotations));
      pidController.reset();
      wristSetpoint = targetPosition;
    }

    double currentPos = wrist.getPosition().in(Rotations);
    double pidOutput = pidController.calculate(currentPos);
    pidOutput = Math.max(-5, Math.min(pidOutput, 5));

    wrist.setVoltage(pidOutput);
  }

  public ScoringSetpoints(
      ElevatorSubsystem elevator, WristSubsystem wrist, ScoringSetpoint position) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.targetPosition = position;
    boolean isAlgae = position.name.equals("BARGE") || position.name.contains("Algae");
    upAngle = isAlgae ? Rotations.of(0.0) : Rotations.of(0.38);
    addRequirements(elevator, wrist);
  }

  @Override
  public void initialize() {
    pidController.setTolerance(0.02);
  }

  @Override
  public void execute() {
    double elevatorRotations = elevator.getPosition().in(Rotations);

    elevator.positionMM(Rotations.of(targetPosition.elevatorHeight));

    if ((targetPosition.elevatorHeight > elevatorRotations
            && elevatorRotations < ElevatorConstants.kElevatorBarLowerLimit.in(Rotations))
        || (targetPosition.elevatorHeight < elevatorRotations
            && elevatorRotations > ElevatorConstants.kElevatorBarUpperLimit.in(Rotations))) {
      setWristAngle(upAngle);
    } else if (!targetPosition.lateWrist) {
      setWristAngle(Rotations.of(targetPosition.wristAngle));
    } else if (Math.abs(targetPosition.elevatorHeight - elevatorRotations) <= 0.2) {
      setWristAngle(Rotations.of(targetPosition.wristAngle));
    } else {
      setWristAngle(upAngle);
    }

    SmartDashboard.putBoolean(
        "ElevatorAtSepoint",
        elevator
            .getPosition()
            .isNear(Rotations.of(targetPosition.elevatorHeight), Rotations.of(0.1)));
    SmartDashboard.putBoolean("WristAtSetpoint", pidController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setPower(0);
    wrist.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return elevator
            .getPosition()
            .isNear(Rotations.of(targetPosition.elevatorHeight), Rotations.of(0.1))
        && pidController.atSetpoint();
  }
}
