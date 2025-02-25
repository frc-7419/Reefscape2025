package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ControlMode;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ScoringSetpoints extends Command {
  public enum ScoringPosition {
    L1("L1", 0, 0.34, false),
    L2("L2", 5, 0.34, true),
    L3("L3", 13, 0.34, true),
    L4("L4", 27.5, 0.25, true),
    HIGH_ALGAE("HIGH_ALGAE", 11, 0.1, false),
    LOW_ALGAE("LOW_ALGAE", 0, 0.1, false),
    BARGE("BARGE", 29.5, 0.25, true);

    private final String name;
    private final double elevatorHeight;
    private final double wristAngle;
    private final boolean lateWrist;

    ScoringPosition(String name, double elevatorHeight, double wristAngle, boolean lateWrist) {
      this.name = name;
      this.elevatorHeight = elevatorHeight;
      this.wristAngle = wristAngle;
      this.lateWrist = lateWrist;
    }
  }

  private final ElevatorSubsystem elevator;
  private final WristSubsystem wrist;
  private final ScoringPosition targetPosition;
  private final Angle upAngle;

  public ScoringSetpoints(
      ElevatorSubsystem elevator, WristSubsystem wrist, ScoringPosition position) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.targetPosition = position;
    upAngle = position.name.equals("BARGE") ? Rotations.of(0.04) : Rotations.of(0.32);
    addRequirements(elevator, wrist);
  }

  @Override
  public void initialize() {
    elevator.switchControlMode(ElevatorSubsystem.ControlMode.PID);
    wrist.switchControlMode(WristSubsystem.ControlMode.PID);
  }

  @Override
  public void execute() {
    double elevatorRotations = elevator.getPosition().in(Rotations);

    elevator.toPosition(Rotations.of(targetPosition.elevatorHeight));

    if ((targetPosition.elevatorHeight > elevatorRotations
            && elevatorRotations < ElevatorConstants.kElevatorBarLowerLimit.in(Rotations))
        || (targetPosition.elevatorHeight < elevatorRotations
            && elevatorRotations > ElevatorConstants.kElevatorBarUpperLimit.in(Rotations))) {
      wrist.toAngle(upAngle);
    } else if (!targetPosition.lateWrist) {
      wrist.toAngle(Rotations.of(targetPosition.wristAngle));
    } else if (Math.abs(targetPosition.elevatorHeight - elevatorRotations) <= 0.2) {
      wrist.toAngle(Rotations.of(targetPosition.wristAngle));
    } else {
      wrist.toAngle(upAngle);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.switchControlMode(ElevatorSubsystem.ControlMode.MANUAL);
    wrist.switchControlMode(WristSubsystem.ControlMode.MANUAL);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
