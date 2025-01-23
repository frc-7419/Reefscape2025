// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.LightSensorSubsystem;
import frc.robot.subsystems.intake.WristIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private final WristIntakeSubsystem wristIntakeSubsystem;
  private final LightSensorSubsystem lightSensorSubsystem;

  /** Creates a new IntakeCoral. */
  public IntakeCoral(WristIntakeSubsystem wristIntakeSubsystem, LightSensorSubsystem lightSensorSubsystem) {
    this.wristIntakeSubsystem = wristIntakeSubsystem;
    this.lightSensorSubsystem = lightSensorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristIntakeSubsystem, lightSensorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristIntakeSubsystem.coastCoral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristIntakeSubsystem.setCoralPower(Constants.IntakeCoralConstants.IntakeCoralPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristIntakeSubsystem.setCoralPower(0);
    wristIntakeSubsystem.brakeCoral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lightSensorSubsystem.coralThere();
  }
}
