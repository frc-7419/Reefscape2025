// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.LightSensorSubsystem;
import frc.robot.subsystems.intake.WristIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeWithBeamBreak extends Command {
  private final WristIntakeSubsystem wristIntakeSubsystem;

  public IntakeWithBeamBreak(
      WristIntakeSubsystem wristIntakeSubsystem, LightSensorSubsystem lightSensorSubsystem) {
    this.wristIntakeSubsystem = wristIntakeSubsystem;

    addRequirements(wristIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristIntakeSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristIntakeSubsystem.setPower(Constants.IntakeCoralConstants.intakeCoralPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristIntakeSubsystem.setPower(0);
    wristIntakeSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wristIntakeSubsystem.beamBreakisTriggered();
  }
}
