package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ScoringSetpoints extends Command {
  public enum ScoringPosition {
    L1(0, 0.34, false),
    L2(5, 0.34, true),
    L3(13, 0.34, true),
    L4(27.5, 0.25, true),
    HIGH_ALGAE(11, 0.1, false),
    LOW_ALGAE(0, 0.1, false),
    BARGE(28.7, 0.32, true);

    private final double elevatorHeight;
    private final double wristAngle;
    private final boolean lateWrist;

    ScoringPosition(double elevatorHeight, double wristAngle, boolean lateWrist) {
      this.elevatorHeight = elevatorHeight;
      this.wristAngle = wristAngle;
      this.lateWrist = lateWrist;
    }
  }

  private final ElevatorSubsystem elevator;
  private final WristSubsystem wrist;
  private final ScoringPosition targetPosition;

  public ScoringSetpoints(
      ElevatorSubsystem elevator, WristSubsystem wrist, ScoringPosition position) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.targetPosition = position;
    addRequirements(elevator, wrist);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double elevatorRotations = elevator.getPosition().in(Rotations);

    elevator.toPosition(Rotations.of(targetPosition.elevatorHeight));

    if ((targetPosition.elevatorHeight > elevatorRotations
            && elevatorRotations < ElevatorConstants.kElevatorBarLowerLimit.in(Rotations))
        || (targetPosition.elevatorHeight < elevatorRotations
            && elevatorRotations > ElevatorConstants.kElevatorBarUpperLimit.in(Rotations))) {
      wrist.toAngle(WristConstants.kElevatorSafeWristAngle);
    } else if (!targetPosition.lateWrist) {
      wrist.toAngle(Rotations.of(targetPosition.wristAngle));
    } else if (Math.abs(targetPosition.elevatorHeight - elevatorRotations) <= 0.5) {
      wrist.toAngle(Rotations.of(targetPosition.wristAngle));
    } else {
      wrist.toAngle(WristConstants.kElevatorSafeWristAngle);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
