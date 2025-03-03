package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;

public class IntakeWithBeamBreak extends Command {
  private final WristIntakeSubsystem wristIntakeSubsystem;
  private final Debouncer debouncer = new Debouncer(0.02); // how long beam must be broken

  public IntakeWithBeamBreak(WristIntakeSubsystem wristIntakeSubsystem) {
    this.wristIntakeSubsystem = wristIntakeSubsystem;

    addRequirements(wristIntakeSubsystem);
  }

  @Override
  public void initialize() {
    wristIntakeSubsystem.coast();
    wristIntakeSubsystem.setVoltage(Constants.IntakeCoralConstants.intakeCoralVoltage);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    wristIntakeSubsystem.setPower(0);
    wristIntakeSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    // return debouncer.calculate(!wristIntakeSubsystem.beamBreakisTriggered());
    return !wristIntakeSubsystem.beamBreakisTriggered();
  }
}
