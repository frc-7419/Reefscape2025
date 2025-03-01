// package frc.robot.commands;

// import static edu.wpi.first.units.Units.Rotations;

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.constants.Constants.ElevatorConstants;
// import frc.robot.subsystems.elevator.ElevatorSubsystem;
// import frc.robot.subsystems.wrist.WristSubsystem;

public class ScoringSetpoints extends Command {
  public enum ScoringSetpoint {
    L1("L1", 0, 0.34, false),
    L2("L2", 5, 0.34, true),
    L3("L3", 13, 0.34, true),
    L4("L4", 27.5, 0.25, true),
    HIGH_ALGAE("HIGH_ALGAE", 11, 0.1, false),
    LOW_ALGAE("LOW_ALGAE", 0, 0.1, false),
    BARGE("BARGE", 29.5, 0.25, true),
    HOME("HOME", 0, 0.42, true);

    public final String name;
    public final double elevatorHeight;
    public final double wristAngle;
    public final boolean lateWrist;

    ScoringSetpoint(String name, double elevatorHeight, double wristAngle, boolean lateWrist) {
      this.name = name;
      this.elevatorHeight = elevatorHeight;
      this.wristAngle = wristAngle;
      this.lateWrist = lateWrist;
    }
  }

  private final ElevatorSubsystem elevator;
  private final WristSubsystem wrist;
  private final ScoringSetpoint targetPosition;
  private final Angle upAngle;

  public ScoringSetpoints(
      ElevatorSubsystem elevator, WristSubsystem wrist, ScoringSetpoint position) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.targetPosition = position;
    upAngle = position.name.equals("BARGE") ? Rotations.of(0.04) : Rotations.of(0.32);
    addRequirements(elevator, wrist);
  }

//   @Override
//   public void initialize() {
//   }

//   @Override
//   public void execute() {
//     double elevatorRotations = elevator.getPosition().in(Rotations);

//     elevator.toPosition(Rotations.of(targetPosition.elevatorHeight));

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
    elevator.setPower(0);
    wrist.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return elevator.atSetpoint() && wrist.atSetpoint();
  }
}
