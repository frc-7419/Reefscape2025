package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.DrivetrainConstants;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/** A command that attempts to detect and counter robot tipping. */
public class AntiTip extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Command elevatorPositionCommand;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** Creates a new AntiTip. */
  public AntiTip(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevator) {
    this.drivetrain = drivetrain;

    elevatorPositionCommand = elevator.setPosition(Rotations.of(0));

    addRequirements(drivetrain, elevator);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {}

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    double pitch = drivetrain.getPigeon2().getPitch().getValue().in(Radians);
    double roll = drivetrain.getPigeon2().getRoll().getValue().in(Radians);

    if (isTipping(pitch, roll, RobotConstants.kTippingThresholdDeg)) {
      elevatorPositionCommand.schedule();

      double[] counterVec = calculateCounterVector(pitch, roll);

      double vx = counterVec[0];
      double vy = counterVec[1];

      drivetrain.setControl(drive.withVelocityX(vx).withVelocityY(vy));
    } else {
      elevatorPositionCommand.cancel();
    }
  }

  /** Called once the command ends or is interrupted. */
  @Override
  public void end(boolean interrupted) {
    elevatorPositionCommand.cancel();
  }

  /** Returns true when the command should end (never, in this example). */
  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean isTipping(double pitchDegrees, double rollDegrees, double thresholdDeg) {
    return (Math.abs(pitchDegrees) > thresholdDeg) || (Math.abs(rollDegrees) > thresholdDeg);
  }

  /**
   * Computes a velocity vector (vx, vy) to drive in the opposite direction of the current tilt,
   * attempting to counteract tipping.
   *
   * <p>Heuristic. Not physically accurate but should work well enough.
   *
   * @param pitch The current pitch in radians (nose up/down).
   * @param roll The current roll in radians (side-to-side).
   * @return A double array {vx, vy}, in meters/seconds.
   */
  private double[] calculateCounterVector(double pitch, double roll) {
    double tiltMagnitude = Math.sqrt(pitch * pitch + roll * roll);

    double tiltAngle = Math.atan2(roll, pitch);

    double counterAngle = tiltAngle + Math.PI;

    double scaleFactor = 0.5;
    double desiredSpeed = tiltMagnitude * RobotConstants.kComHeight * scaleFactor;

    desiredSpeed = Math.min(desiredSpeed, DrivetrainConstants.kMaxVelocity.in(MetersPerSecond));

    double vx = desiredSpeed * Math.cos(counterAngle);
    double vy = desiredSpeed * Math.sin(counterAngle);

    return new double[] {vx, vy};
  }
}
