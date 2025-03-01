// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeWithBeamBreak extends Command {
  private final WristIntakeSubsystem wristIntakeSubsystem;
  private final Debouncer debouncer = new Debouncer(0.2); // how long beam must be broken

  public IntakeWithBeamBreak(WristIntakeSubsystem wristIntakeSubsystem) {
    this.wristIntakeSubsystem = wristIntakeSubsystem;

    addRequirements(wristIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristIntakeSubsystem.coast();
    wristIntakeSubsystem.setPower(Constants.IntakeCoralConstants.intakeCoralPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristIntakeSubsystem.setPower(0);
    wristIntakeSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return debouncer.calculate(wristIntakeSubsystem.beamBreakisTriggered());
  }
}
