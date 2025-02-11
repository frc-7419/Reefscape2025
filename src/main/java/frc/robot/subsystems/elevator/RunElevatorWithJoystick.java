package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevatorWithJoystick extends Command {
  ElevatorSubsystem elevatorSubsystem;
  DoubleSupplier doubleSupplier;
  /** Creates a new RunElevatorWithJoystick. */
  public RunElevatorWithJoystick(ElevatorSubsystem elevatorSubsystem, DoubleSupplier doubleSupplier) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.doubleSupplier = doubleSupplier;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.coast();
  }

  @Override
  public void execute() {
    elevatorSubsystem.setPower(doubleSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.brake();
    elevatorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
